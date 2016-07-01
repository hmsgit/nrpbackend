"""
Takes care of the operations on the Collab portal (includes Collab and Document clients)
"""

import logging
import os
import tempfile
import shutil
import re

from bbp_client.oidc.client import BBPOIDCClient
from bbp_client.collab_service.client import Client as CollabClient
from hbp_nrp_backend.collab_interface import NRPServicesUploadException
from bbp_client.document_service.client import Client as DocumentClient
from bbp_client.document_service.exceptions import DocException
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_backend.rest_server.__CollabContext import get_or_raise as get_or_raise_collab_context
from hbp_nrp_backend import hbp_nrp_backend_config

__author__ = "Daniel Peppicelli"
logger = logging.getLogger(__name__)


class NeuroroboticsCollabClient(object):
    """
    Helper class to perform neurorobotics operations on the platform (Collab, OIDC or
    Document services)
    """

    EXPERIMENT_CONFIGURATION_FILE_NAME = "experiment_configuration.xml"
    EXPERIMENT_CONFIGURATION_MIMETYPE = "application/hbp-neurorobotics+xml"
    SDF_WORLD_MIMETYPE = "application/hbp-neurorobotics.sdf.world+xml"
    SDF_ROBOT_MIMETYPE = "application/hbp-neurorobotics.sdf.robot+xml"
    BRAIN_PYNN_MIMETYPE = "application/hbp-neurorobotics.brain+python"
    # BIBI stands for Brain Interface and Body Integration.
    # It encompasses brain model and transfer functions
    # while it holds a reference to the body model.
    BIBI_CONFIGURATION_FILE_NAME = "bibi_configuration.xml"
    BIBI_CONFIGURATION_MIMETYPE = "application/hbp-neurorobotics.bibi+xml"
    TRANSFER_FUNCTIONS_FILE_NAME = "transfer_functions.py"
    TRANSFER_FUNCTIONS_PY_MIMETYPE = "application/hbp-neurorobotics.tfs+python"
    PNG_IMAGE_MIMETYPE = "image/png"

    def __init__(self, token, context_id):
        """
        Creates the wrapper client
        :param token: The OIDC token used by the requester
        :param context_id: The Collab context
        """
        self.__context_id = context_id
        # Retrieves OIDC, Document server and Collab urls from configuration object
        cfg = hbp_nrp_backend_config.COLLAB_CLIENT_CONFIG
        # Getting an OIDC client to then get a Collab and a Document client.
        oidc = BBPOIDCClient.bearer_auth(cfg['oidc_server'], token)

        self.__collab_client = CollabClient(cfg['collab_server'], oidc, 0)
        self.__collab_client.set_collab_id_by_context(context_id)

        self.__document_client = DocumentClient(cfg['document_server'], oidc)
        self.__project = self.__document_client.get_project_by_collab_id(
            self.__collab_client.collab_id
        )

        self.__collab_context = get_or_raise_collab_context(self.__context_id)

    def get_mimetype(self, file_path):
        """
        Returns the mimetype of the given file
        :param file_path: file path for which the mimetype should be returned
        :return: the mimetype of the given file
        """
        file_name = os.path.basename(file_path)
        mimetype = None
        if file_name == self.EXPERIMENT_CONFIGURATION_FILE_NAME:
            mimetype = self.EXPERIMENT_CONFIGURATION_MIMETYPE
        elif file_name == self.BIBI_CONFIGURATION_FILE_NAME:
            mimetype = self.BIBI_CONFIGURATION_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.sdf') and\
             (self.__find_regexp(file_path, r'<world', re.I)):
            mimetype = self.SDF_WORLD_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.sdf') and\
             (self.__find_regexp(file_path, r'<model', re.I)):
            mimetype = self.SDF_ROBOT_MIMETYPE
        elif (self.__find_regexp(file_path, r'import\s+pyNN|from\s+PyNN\s+import')):
            mimetype = self.BRAIN_PYNN_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.py'):
            mimetype = self.TRANSFER_FUNCTIONS_PY_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.png'):
            mimetype = self.PNG_IMAGE_MIMETYPE
        return mimetype

    @staticmethod
    def __find_regexp(file_path, expr, flags=0):
        """
        Returns True if expr is found in file
        :param file_path: file path where to look for the expression
        :param expr: expression to find in file
        :return: True if found
        """
        with open(file_path, 'r') as content:
            for line in content:
                if re.search(expr, line, flags):
                    return True

        return False

    def get_context_app_name(self):
        """
        Returns the name of the navigation item whose context has been
         used to create this object
        :return: the name of the navigation item whose context has been
         used to create this object
        """
        children = self.__collab_client.get_current_tree()['children']
        return [child for child in children if child['context'] == self.__context_id][0]['name']

    def generate_unique_folder_name(self, base_name):
        """
        Makes a candidate folder name unique
        (no duplicate in the current Collab storage space)
        :param base_name: A base name
        :return: A unique folder name based on the given base name
        """
        suffix = 0
        original_base_name = base_name
        existing_folders = self.__document_client.listdir(
            self.__document_client.get_path_by_id(self.__project['_uuid']))
        while base_name in existing_folders:
            base_name = original_base_name + "_" + str(suffix)
            suffix += 1
        return base_name

    def clone_experiment_template_to_collab(self, folder_name, exp_configuration):
        """
        Takes an experiment template and clones it into the storage space of the current Collab
        :param folder_name: The name of the folder to create in the storage space
        :param exp_configuration: The experiment configuration file of the template to clone
        :return: The UUID of the created folder
        """
        logger.debug(
            "Sync experiment configuration file " +
            exp_configuration +
            " to collab folder " + folder_name
        )
        self.__document_client.chdir(self.__document_client.get_path_by_id(self.__project['_uuid']))
        created_folder_uuid = self.__document_client.mkdir(folder_name)

        logger.debug("Creating a temporary directory where flattened files will go")
        raise_upload_exception = False
        with _FlattenedExperimentDirectory(exp_configuration) as temporary_folder:
            logger.debug("Uploading the flattened experiment files to the Collab")
            for filename in os.listdir(temporary_folder):
                filepath = os.path.join(temporary_folder, filename)
                mimetype = self.get_mimetype(filepath)
                try:
                    self.__document_client.upload_file(
                        filepath,
                        os.path.join(folder_name, filename),
                        mimetype
                    )
                except DocException as e:
                    # The clone operation is aborted,
                    # the create folder is removed from the Collab storage
                    self.__document_client.rmdir(folder_name, force=True)
                    raise_upload_exception = True
                    break

        if raise_upload_exception:
            # the front-end is notified of the upload failure
            raise NRPServicesUploadException(e.message)

        return created_folder_uuid

    def clone_experiment_template_from_collab(self, collab_folder_uuid):
        """
        Takes a collab folder and clones all the files in a temporary folder. The caller has
        then the responsability of managing this folder.
        :param collab_folder_uuid: The UUID of the document service folder where the experiment is
            saved
        :return: A dictionary containing the various path of the cloned elements.
        """
        temp_directory = tempfile.mkdtemp()
        experiment_path = {}
        logger.debug(
            "Sync experiment configuration files (.xml and .sdf) from collab folder " +
            collab_folder_uuid +
            " to local folder " + temp_directory
        )
        collab_folder_path = self.__document_client.get_path_by_id(collab_folder_uuid)
        for filename in self.__document_client.listdir(collab_folder_path):
            filepath = collab_folder_path + '/' + filename
            attr = self.__document_client.get_standard_attr(filepath)
            if (attr['_entityType'] == 'file'):
                localpath = os.path.join(temp_directory, filename)
                self.__document_client.download_file(
                    filepath, localpath)
                if '_contentType' in attr:
                    if attr['_contentType'] == self.EXPERIMENT_CONFIGURATION_MIMETYPE:
                        experiment_path['experiment_conf'] = localpath
                    elif attr['_contentType'] == self.SDF_WORLD_MIMETYPE:
                        experiment_path['environment_conf'] = localpath
                    elif attr['_contentType'] == self.SDF_ROBOT_MIMETYPE:
                        experiment_path['robot_model'] = localpath
        return experiment_path

    def clone_experiment_template_from_collab_context(self):
        """
        Takes a collab folder and clones all the files in a temporary folder. The caller has
        then the responsibility of managing the returned folder.
        :return: A dictionary containing the various path of the cloned elements.
        """
        return self.clone_experiment_template_from_collab(
            self.__collab_context.experiment_folder_uuid
        )

    def clone_file_from_collab(self, collab_folder_uuid, mimetype):
        """
        Takes a collab folder and clones a file according to a given mimetype in a
        temporary folder.
        The caller has then the responsability of managing this folder.

        :param collab_folder_uuid: The UUID of the document service folder where the experiment is
            saved
        :param mimetype: The mimetype of the file to clone
        :return: A string containing the path of the cloned bibi configuration file.
        """
        temp_directory = tempfile.mkdtemp()
        logger.debug(
            "Sync file with mimetype " + mimetype + " from collab folder " +
            collab_folder_uuid +
            " to local folder " + temp_directory
        )
        collab_folder_path = self.__document_client.get_path_by_id(collab_folder_uuid)
        for filename in self.__document_client.listdir(collab_folder_path):
            filepath = collab_folder_path + '/' + filename
            attr = self.__document_client.get_standard_attr(filepath)
            if (
                (attr['_entityType'] == 'file') and ('_contentType' in attr) and
                (attr['_contentType'] == mimetype)
            ):
                localpath = os.path.join(temp_directory, filename)
                self.__document_client.download_file(filepath, localpath)
                return localpath

        return None

    def clone_file_from_collab_context(self, mimetype):
        """
        Takes a collab folder and clones a file according to a given mimetype in a
        temporary folder.
        The caller has then the responsability of managing the created folder.

        :param mimetype: The mimetype of the file to clone
        :return: A string containing the path of the cloned bibi configuration file.
        """
        return self.clone_file_from_collab(self.__collab_context.experiment_folder_uuid, mimetype)

    def get_first_file_path_with_mimetype(self, mimetype, default_filename):
        """
        Return the full path (on the collab) of the first file found with a
        given mimetype. If nothing is found, a path with a given
        "default_filename" will be returned.
        :param mimetype: The mimetype to find
        :param default_filename: A default filename to return along with the
        collab path when nothing is found.
        """
        collab_folder_path = \
            self.__document_client.get_path_by_id(self.__collab_context.experiment_folder_uuid)
        for filename in self.__document_client.listdir(collab_folder_path):
            filepath = collab_folder_path + '/' + filename
            attr = self.__document_client.get_standard_attr(filepath)
            if '_contentType' in attr and attr['_contentType'] == mimetype:
                return filepath

        default_filepath = os.path.join(collab_folder_path, default_filename)
        return default_filepath

    def replace_file_content_in_collab(self, content, mimetype, default_filename):
        """
        Replace the content of the first file found with a given mimetype by some
        new content

        :param content: new content (in UTF-8)
        :param mimetype: mimetype of the file to search for
        :param default_filename: if no file with the given mimetype is found, create
        a new file with this default filename.
        """
        filepath = self.get_first_file_path_with_mimetype(mimetype, default_filename)
        if self.__document_client.exists(filepath):
            self.__document_client.remove(filepath)
        self.__document_client.upload_string(content, filepath, mimetype)

    def write_file_with_content_in_collab(self, content, mimetype, filename):
        """
        Create or override a file in the collab with a given content, mimetype and filename

        :param content: content of the file (in UTF-8).
        :param mimetype: mimetype of the file.
        :param filename: self explaining.
        """
        collab_folder_path = \
            self.__document_client.get_path_by_id(self.__collab_context.experiment_folder_uuid)
        filepath = os.path.join(collab_folder_path, filename)
        if self.__document_client.exists(filepath):
            self.__document_client.remove(filepath)
        self.__document_client.upload_string(content, filepath, mimetype)

    def populate_subfolder_in_collab(self, foldername, files, mimetype=None):
        """
        Create or overwrite a subfolder of the collab experiment folder
        and populate it with files specified by means of their paths

        :param foldername: self explaining.
        :param files: an array of dictionaries {name: 'my_file.txt', temporary_path: 'my_file_path'}
        :param mimetype: mimetype common to all files
        :return the created folder uuid
        """
        collab_experiment_folderpath = \
            self.__document_client.get_path_by_id(self.__collab_context.experiment_folder_uuid)
        self.__document_client.chdir(collab_experiment_folderpath)
        folderpath = os.path.join(collab_experiment_folderpath, foldername)
        if self.__document_client.exists(folderpath):
            self.__document_client.rmdir(foldername)
        created_folder_uuid = self.__document_client.mkdir(foldername)
        try:
            for f in files:
                self.__document_client.upload_file(
                    f.temporary_path,
                    os.path.join(foldername, f.name),
                    mimetype
                )
        except DocException as e:
            raise NRPServicesUploadException(e.message)

        return created_folder_uuid

    def download_file_from_collab(self, file_path, dst_path=None):
        """
        Downloads the file with filepath from the collab.

        :param file_path: the collab path of the file to download
        :param dst_path: the destination where the file has to be saved (optional)
        :return the contents of the file as a string or the path if dst_path is provided
        """
        return self.__document_client.download_file(file_path, dst_path)


class _FlattenedExperimentDirectory(object):
    """
    Helper context class. It takes all the files useful to describe an
    experiment and puts them at the root of a temporary directory.
    All the links in these files are updated.
    """

    def __init__(self, exp_configuration):
        """
        Constructor. Does nothing more that saving the exp_configuration parameter.
        :param exp_configuration: The experiment configuration file of the template to clone.
        """
        self.__exp_configuration = exp_configuration
        self.__temp_directory = None
        # exp_configuration may look like:
        # $NRP_MODELS/ExDConf/ExDBraitenbergHuskySBC.xml
        # We extract: $NRP_MODELS
        self.__models_folder = os.path.dirname(os.path.dirname(exp_configuration))

    def __enter__(self):
        """
        Performs the flattening.
        :return: The temporary directory where the flattened experiment is located.
        """
        self.__temp_directory = tempfile.mkdtemp()
        logger.debug("Using temporary directory " +
                     self.__temp_directory +
                     " to flatten experiment template")

        # Get the experiment configuration file as a DOM object
        with open(self.__exp_configuration) as e:
            experiment_dom = exp_conf_api_gen.CreateFromDocument(e.read())

        # Get the experiment image and copy it into the flattened experiment directory
        exd_split = os.path.split(self.__exp_configuration)
        img_file = os.path.join(exd_split[0], exd_split[-1].split('.')[0]) + '.png'
        shutil.copyfile(img_file, os.path.join(self.__temp_directory, os.path.basename(img_file)))
        # Get the SDF file path and copy it into the flattened experiment directory
        sdf_file = os.path.join(self.__models_folder, experiment_dom.environmentModel.src)
        shutil.copyfile(sdf_file, os.path.join(self.__temp_directory, os.path.basename(sdf_file)))
        # Update the experiment configuration file with the new paths
        experiment_dom.environmentModel.src = os.path.basename(sdf_file)
        # Get the BIBI file path and copy it into the flattened experiment directory
        bibi_configuration_file = os.path.join(self.__models_folder, experiment_dom.bibiConf.src)
        # Update the BIBI configuration file with the new paths
        experiment_dom.bibiConf.src = NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME
        flattened_exp_configuration_file = \
            os.path.join(
                self.__temp_directory,
                NeuroroboticsCollabClient.EXPERIMENT_CONFIGURATION_FILE_NAME
            )
        # save experiment configuration file
        with open(flattened_exp_configuration_file, "w") as f:
            f.write(experiment_dom.toDOM().toprettyxml())

        # flatten bibi configuration file and save
        self.__flatten_bibi_configuration(bibi_configuration_file)
        return self.__temp_directory

    def __flatten_bibi_configuration(self, bibi_configuration_file):
        """
        Flatten the bibi configuration with respect to transfer function
        external references:
        Copy all external dependencies (tf python scripts) for transfer functions in
        a temporary 'flattened' folder
        Update the file paths of the corresponding transfer functions
        in the bibi configuration file
        Copy the 'flattened' bibi configuration file to the above folder
        :param bibi_configuration_file: path to the bibi configuration file
        of the template experiment
        """
        # Get the bibi configuration file as a DOM object
        with open(bibi_configuration_file) as b:
            bibi_configuration_dom = bibi_api_gen.CreateFromDocument(b.read())

        # Get the robot file path and copy it into the flattened experiment directory
        robot_sdf_file = os.path.join(self.__models_folder, bibi_configuration_dom.bodyModel)
        robot_sdf_file_name = os.path.basename(robot_sdf_file)
        shutil.copyfile(robot_sdf_file, os.path.join(self.__temp_directory, robot_sdf_file_name))
        bibi_configuration_dom.bodyModel = bibi_api_gen.SDF_Filename(robot_sdf_file_name)
        # Get the PyNN file path and copy it into the flattened experiment directory
        brain_file = os.path.join(self.__models_folder, bibi_configuration_dom.brainModel.file)
        brain_file_name = os.path.basename(brain_file)
        shutil.copyfile(brain_file, os.path.join(self.__temp_directory, brain_file_name))
        bibi_configuration_dom.brainModel.file = bibi_api_gen.Python_Filename(brain_file_name)

        # Copy 'flattened' dependencies to temporary folder
        # and update file paths of TF python scripts
        for tf in bibi_configuration_dom.transferFunction:
            if hasattr(tf, "src") and tf.src:
                tf_file = os.path.join(self.__models_folder, tf.src)
                tf.src = os.path.basename(tf.src)
                shutil.copyfile(
                    tf_file,
                    os.path.join(self.__temp_directory, tf.src)
                )
        flattened_bibi_configuration_file = os.path.join(
            self.__temp_directory,
            NeuroroboticsCollabClient.BIBI_CONFIGURATION_FILE_NAME
        )
        with open(flattened_bibi_configuration_file, "w") as b:
            b.write(bibi_configuration_dom.toDOM().toprettyxml())

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Cleanup the temporary folder containing the flattened experiment
        :param exc_type: Exception type if any (None otherwise).
        :param exc_value: Exception value if any (None otherwise).
        :param traceback: Exception traceback if any (None otherwise).
        """
        if (not (exc_type or exc_value or traceback)):
            logger.debug("Clean up temporary directory " + self.__temp_directory)
            shutil.rmtree(self.__temp_directory)
