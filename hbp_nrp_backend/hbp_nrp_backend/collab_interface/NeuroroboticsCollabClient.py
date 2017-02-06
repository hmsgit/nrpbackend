"""
Takes care of the operations on the Collab portal (includes Collab and Document clients)
"""

import logging
import os
import tempfile
import shutil
import re
from threading import Thread
from pyxb import ValidationError
from xml.sax import SAXParseException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_backend.rest_server.__ExperimentService import \
    ErrorMessages
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

    EXPERIMENT_CONFIGURATION_FILE_NAME = "experiment_configuration.exc"
    EXPERIMENT_CONFIGURATION_MIMETYPE = "application/hbp-neurorobotics+xml"
    SDF_WORLD_MIMETYPE = "application/hbp-neurorobotics.sdf.world+xml"
    # potential file name of sdf world
    SDF_WORLD_FILE_NAME = "virtual_room.sdf"
    SDF_ROBOT_MIMETYPE = "application/hbp-neurorobotics.sdf.robot+xml"
    BRAIN_PYNN_MIMETYPE = "application/hbp-neurorobotics.brain+python"
    # potential file name of the brain
    BRAIN_PYNN_FILE_NAME = "braitenberg.py"
    # BIBI stands for Brain Interface and Body Integration.
    # It encompasses brain model and transfer functions
    # while it holds a reference to the body model.
    BIBI_CONFIGURATION_FILE_NAME = "bibi_configuration.bibi"
    BIBI_CONFIGURATION_MIMETYPE = "application/hbp-neurorobotics.bibi+xml"
    TRANSFER_FUNCTIONS_FILE_NAME = "transfer_functions.py"
    TRANSFER_FUNCTIONS_PY_MIMETYPE = "application/hbp-neurorobotics.tfs+python"
    PNG_IMAGE_MIMETYPE = "image/png"
    STATE_MACHINE_PY_MIMETYPE = "application/hbp-neurorobotics.sm+python"

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
        self.__collab_context = get_or_raise_collab_context(self.__context_id)
        self.__project = None

        def thread_project():
            """
            Function to call get_project_by_collab_id and set the variable __project
            """
            self.__project = self.__document_client.get_project_by_collab_id(
                self.__collab_client.collab_id)
        self._project_thread = Thread(target=thread_project)
        self._project_thread.start()
        self.collab_folder_path = {}

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
             (self.find_regexp(file_path, r'<world', re.I)):
            mimetype = self.SDF_WORLD_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.sdf') and\
             (self.find_regexp(file_path, r'<model', re.I)):
            mimetype = self.SDF_ROBOT_MIMETYPE
        elif (self.find_regexp(file_path, r'import\s+pyNN|from\s+PyNN\s+import') or
              self.find_regexp(
                  file_path,
                  r'import\s+hbp_nrp_cle\.brainsim|from\s+hbp_nrp_cle\.brainsim\s+import')):
            mimetype = self.BRAIN_PYNN_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.py'):
            mimetype = self.TRANSFER_FUNCTIONS_PY_MIMETYPE
        elif (os.path.splitext(file_name)[1] == '.png'):
            mimetype = self.PNG_IMAGE_MIMETYPE
        return mimetype

    @staticmethod
    def find_regexp(file_path, expr, flags=0):
        """
        Returns a match if expr is found in file, False otherwise
        :param file_path: file path where to look for the expression
        :param expr: expression to find in file
        :return: match if found
        """
        with open(file_path, 'r') as content:
            for line in content:
                match = re.search(expr, line, flags)
                if match:
                    return match

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
            self._get_path_by_id(self._get_project()['_uuid']))
        while base_name in existing_folders:
            base_name = original_base_name + "_" + str(suffix)
            suffix += 1
        return base_name

    def _get_project(self):
        """
        Returns the self.__project variable once it's ready
        :return: self.__project
        """
        if not self.__project:
            self._project_thread.join()
        return self.__project

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
        self.__document_client.chdir(self._get_path_by_id(self._get_project()['_uuid']))
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
        threads = []
        collab_folder_path = self._get_path_by_id(collab_folder_uuid)
        for filename in self.__document_client.listdir(collab_folder_path):
            if (filename != 'edit.lock'):
                filepath = collab_folder_path + '/' + filename
                attr = self.__document_client.get_standard_attr(filepath)
                if (attr['_entityType'] == 'file'):
                    localpath = os.path.join(temp_directory, filename)
                    t = Thread(target=self.__document_client.download_file_by_id,
                               args=(attr['_uuid'], localpath))
                    t.start()
                    threads.append(t)
                    if '_contentType' in attr:
                        if attr['_contentType'] == self.EXPERIMENT_CONFIGURATION_MIMETYPE:
                            experiment_path['experiment_conf'] = localpath
                        elif attr['_contentType'] == self.SDF_WORLD_MIMETYPE:
                            experiment_path['environment_conf'] = localpath
                        elif attr['_contentType'] == self.SDF_ROBOT_MIMETYPE:
                            experiment_path['robot_model'] = localpath
        for t in threads:
            t.join()
        logger.info(experiment_path)
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

    def clone_file_from_collab(self, collab_folder_uuid, mimetype, potential_filename):
        """
        Takes a collab folder and clones a file according to a given mimetype in a
        temporary folder.
        The caller has then the responsability of managing this folder.

        :param collab_folder_uuid: The UUID of the document service folder where the experiment is
            saved
        :param mimetype: The mimetype of the file to clone
        :param potential_filename: The potential file name of the file that wants to be cloned
        :return: A tuple containing:
                     The local path of the cloned file,
                     the remote path of the file.
        """
        temp_directory = tempfile.mkdtemp()
        logger.debug(
            "Sync file with mimetype " + mimetype + " from collab folder " +
            collab_folder_uuid +
            " to local folder " + temp_directory
        )
        collab_folder_path = self._get_path_by_id(collab_folder_uuid)

        def download_file(attr, localpath):
            """
            Download a file if it is the correct mimetype
            :param localpath: where to save the file
            :return True/False depending on if the file was downloaded
            """
            if (
                (attr['_entityType'] == 'file') and ('_contentType' in attr) and
                (attr['_contentType'] == mimetype)
            ):
                self.__document_client.download_file_by_id(attr['_uuid'], localpath)
                return True
            return False

        try:
            filepath = collab_folder_path + '/' + potential_filename
            attr = self.__document_client.get_standard_attr(filepath)
            localpath = os.path.join(temp_directory, potential_filename)
            if download_file(attr, localpath):
                return localpath, filepath
        except (DocException, OSError):
            # file doesn't exist
            pass

        # either file did not exist or it wasn't the correct mimetype
        # search in collab
        for filename in self.__document_client.listdir(collab_folder_path):
            filepath = collab_folder_path + '/' + filename
            attr = self.__document_client.get_standard_attr(filepath)
            localpath = os.path.join(temp_directory, filename)
            if download_file(attr, localpath):
                return localpath, filepath

    def _get_path_by_id(self, uuid):
        """
        Returns the folder path from an id
        :param uuid: the uuid of the folder wanted
        :return folder path
        """
        if uuid not in self.collab_folder_path:
            self.collab_folder_path[uuid] = self.__document_client.get_path_by_id(uuid)
        return self.collab_folder_path[uuid]

    def clone_file_from_collab_context(self, mimetype, filename):
        """
        Takes a collab folder and clones a file according to a given mimetype in a
        temporary folder.
        The caller has then the responsability of managing the created folder.

        :param mimetype: The mimetype of the file to clone
        :param filename: The possible name of the file to be cloned
        :param content: Boolean, true if the file contents should be returned
        :return: A string containing the path of the cloned file,
                 the path the cloned file was found and the content
        """
        return self.clone_file_from_collab(self.__collab_context.experiment_folder_uuid,
                                           mimetype,
                                           filename)

    def clone_bibi_file_from_collab_context(self):
        """
        Clones the bibi file from the collab storage
        :return: A BIBIConfiguration object, the path of the cloned file as a string and
                 the remote path where the file was downloaded from.
        """
        clone_return = self.clone_file_from_collab_context(self.BIBI_CONFIGURATION_MIMETYPE,
                                                           self.BIBI_CONFIGURATION_FILE_NAME)
        if clone_return is None:
            raise NRPServicesGeneralException(
                ErrorMessages.EXPERIMENT_BIBI_FILE_NOT_FOUND_404,
                "Experiment bibi file was not found in the collab storage"
            )
        local_filepath, remote_path = clone_return
        return (self._parse_and_check_file_is_valid(local_filepath,
                                                    bibi_api_gen.CreateFromDocument,
                                                    bibi_api_gen.BIBIConfiguration),
                local_filepath, remote_path)

    def clone_exp_file_from_collab_context(self):
        """
        Clones the experiment file from the collab storage
        :return: A ExD_ object and the path of the cloned file as a string and
                 the remote path where the file was downloaded from.
        """
        clone_return = self.clone_file_from_collab_context(self.EXPERIMENT_CONFIGURATION_MIMETYPE,
                                                           self.EXPERIMENT_CONFIGURATION_FILE_NAME)
        if clone_return is None:
            raise NRPServicesGeneralException(
                ErrorMessages.EXPERIMENT_CONF_FILE_NOT_FOUND_404,
                "Experiment xml not found in the collab storage"
            )
        local_filepath, remote_path = clone_return
        return (self._parse_and_check_file_is_valid(local_filepath,
                                                    exp_conf_api_gen.CreateFromDocument,
                                                    exp_conf_api_gen.ExD_),
                local_filepath, remote_path)

    @staticmethod
    def _parse_and_check_file_is_valid(filepath, create_obj_function, instance_type):
        """
        Parses a file and checks if it corresponds to its instance type and
        can be created into its object
        :param filepath: The path of the file
        :param create_obj_function: The function to create the object
        :param instance_type: The required instance type of the file
        :return: An object containing the file content
        """
        with open(filepath) as file_content:
            try:
                file_obj = create_obj_function(file_content.read())
                if not isinstance(file_obj, instance_type):
                    raise NRPServicesGeneralException(
                        "%s configuration file content is not valid." % (filepath),
                        "File not valid"
                    )
            except (ValidationError, SAXParseException) as ve:
                raise NRPServicesGeneralException(
                    "Could not parse file %s due to validation error: %s" % (filepath, str(ve)),
                    "File not valid"
                )
        return file_obj

    def replace_file_content_in_collab(self, content, mimetype, filename):
        """
        Replace the content of a file or if the file does not exist, creates a new file.

        :param content: content (in UTF-8)
        :param mimetype: mimetype of the file
        :param filename: the name or path of the file
        """
        collab_folder_path = self._get_path_by_id(self.__collab_context.experiment_folder_uuid)
        if collab_folder_path not in filename:
            filepath = os.path.join(collab_folder_path, filename)
        else:
            filepath = filename
        try:
            self.__document_client.remove(filepath)
        except (DocException, OSError):
            # it doesn't exist
            pass
        self.__document_client.upload_string(content, filepath, mimetype)

    def find_and_replace_file_in_collab(self, content, mimetype, default_name):
        """
        Finds a file with a given mimetype and overwrites it with new content.
        :param content: content (in UTF-8)
        :param mimetype: the mimetype of the file
        :param default_name: if a file with this mimetype is not found, use this name instead
        """
        collab_folder_path = self._get_path_by_id(self.__collab_context.experiment_folder_uuid)
        found = False
        for filename in self.__document_client.listdir(collab_folder_path):
            filepath = os.path.join(collab_folder_path, filename)
            attr = self.__document_client.get_standard_attr(filepath)
            if '_contentType' in attr and attr['_contentType'] == mimetype:
                # found the correct file
                found = True
                break
        if not found:
            filepath = os.path.join(collab_folder_path, default_name)
        self.replace_file_content_in_collab(content, mimetype, filepath)

    def populate_subfolder_in_collab(self, foldername, files, mimetype=None):
        """
        Create or overwrite a subfolder of the collab experiment folder
        and populate it with files specified by means of their paths

        :param foldername: self explaining.
        :param files: an array of dictionaries {name: 'my_file.txt', temporary_path: 'my_file_path'}
        :param mimetype: mimetype common to all files
        :return the created folder uuid
        """
        collab_experiment_folderpath = self._get_path_by_id(
            self.__collab_context.experiment_folder_uuid
        )
        self.__document_client.chdir(collab_experiment_folderpath)
        try:
            self.__document_client.rmdir(foldername)
        except (DocException, OSError):
            # it doesn't exist
            pass
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

    def add_app_to_nav_menu(self):
        """
        Adds a new app to the navigation menu.
        Will add the same type app as the last one created or if this is the first
        it will add the app titled: Neurorobotics Experiments

        :return json indicating if adding the app was successful
        """
        tree = self.__collab_client.get_current_tree()
        for child in tree["children"]:
            if child['context'] == self.__context_id:
                app_id = child["app_id"]
        return self.__collab_client.add_item(tree['id'], {"app_id": app_id,
                                                          "name": "New Experiment"})


def get_model_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise Exception("Server Error. NRP_MODELS_DIRECTORY not defined.")

    return path


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
        # $NRP_EXPERIMENTS_DIRECTORY/braitenberg_husky/ExDBraitenbergHuskySBC.xml
        # We extract: $NRP_EXPERIMENTS_DIRECTORY
        self.__experiment_folder = os.path.dirname(exp_configuration)
        self.__models_folder = get_model_basepath()

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

        # copy statemachines
        if experiment_dom.experimentControl and experiment_dom.experimentControl.stateMachine:
            for sm in experiment_dom.experimentControl.stateMachine:
                sm_file = os.path.join(self.__experiment_folder, sm.src)
                sm.src = os.path.basename(sm.src)
                shutil.copyfile(sm_file, os.path.join(self.__temp_directory, sm.src))
        # Get the experiment image and copy it into the flattened experiment directory
        img_file = os.path.join(self.__experiment_folder, experiment_dom.thumbnail)
        shutil.copyfile(img_file, os.path.join(self.__temp_directory, os.path.basename(img_file)))
        # Update the experiment thumbnail file with the new path
        experiment_dom.thumbnail = os.path.basename(img_file)
        # Get the SDF file path and copy it into the flattened experiment directory
        sdf_file = os.path.join(self.__models_folder, experiment_dom.environmentModel.src)
        shutil.copyfile(sdf_file, os.path.join(self.__temp_directory, os.path.basename(sdf_file)))
        # Update the experiment configuration file with the new paths
        experiment_dom.environmentModel.src = os.path.basename(sdf_file)
        # Get the BIBI file path and copy it into the flattened experiment directory
        bibi_configuration_file = os.path.join(
            self.__experiment_folder, experiment_dom.bibiConf.src
        )
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

        # copy experiment configuration file
        for conf in experiment_dom.configuration:
            self.__copy_config_file(conf.src)

        # flatten bibi configuration file and save
        self.__flatten_bibi_configuration(bibi_configuration_file)
        return self.__temp_directory

    def __copy_config_file(self, config_file):
        """
        Copy the provided config file to the flattened temporary directory.
        """
        config_filepath = os.path.join(self.__experiment_folder, config_file)
        config_filename = os.path.basename(config_filepath)
        # we copy the config file at the root of the directory
        shutil.copyfile(config_filepath, os.path.join(self.__temp_directory, config_filename))

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

        # Get all config file
        for conf in bibi_configuration_dom.configuration:
            logger.debug("Copying config file {} to collab.".format(conf.src))
            self.__copy_config_file(conf.src)

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
                tf_file = os.path.join(self.__experiment_folder, tf.src)
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
