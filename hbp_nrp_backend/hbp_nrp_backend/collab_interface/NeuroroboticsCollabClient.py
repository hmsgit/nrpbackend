"""
Takes care of the operations on the Collab portal (includes Collab and Document clients)
"""

import logging
import os
import tempfile
import shutil
from bbp_client.oidc.client import BBPOIDCClient
from bbp_client.collab_service.client import Client as CollabClient
from bbp_client.document_service.client import Client as DocumentClient
from hbp_nrp_backend.exd_config.generated import exp_conf_api_gen
from hbp_nrp_backend import hbp_nrp_backend_config

__author__ = "Daniel Peppicelli"
logger = logging.getLogger(__name__)


class NeuroroboticsCollabClient(object):
    """
    Helper class to perform neurorobotics operations on the platform (Collab, OIDC or
    Document services)
    """

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
            self.__collab_client.collab_id)

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

    def clone_experiment_template_to_collab(self, folder_name, exd_configuration):
        """
        Takes an experiment template and clones it into the storage space of the current Collab
        :param folder_name: The name of the folder to create in the storage space
        :param exd_configuration: The experiment configuration file of the template to clone
        """
        logger.debug("Sync models from " + exd_configuration + " to collab folder " + folder_name)
        self.__document_client.chdir(self.__document_client.get_path_by_id(self.__project['_uuid']))
        self.__document_client.mkdir(folder_name)

        logger.debug("Creating a temporary directory where flattened files will go")
        with _FlattenedExperimentDirectory(exd_configuration) \
                as temp_flattened_exp_configuration_folder:
            logger.debug("Uploading the flattened experiment to the collab")
            for filename in os.listdir(temp_flattened_exp_configuration_folder):
                self.__document_client.upload_file(
                    os.path.join(temp_flattened_exp_configuration_folder, filename),
                    folder_name + '/' + filename)


class _FlattenedExperimentDirectory(object):
    """
    Helper context class. It takes all the files useful to describe an
    experiment (ExdConfiguration and the SDF environment file for the moment)
    and puts them at the root of a temporary directory. All the links in these
    files are updated.
    """

    def __init__(self, exd_configuration):
        """
        Constructor. Does nothing more that saving the exd_configuration parameter.
        :param exd_configuration: The experiment configuration file of the template to clone.
        """
        self.__exd_configuration = exd_configuration
        self.__temp_directory = None

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
        with open(self.__exd_configuration) as exd_file:
            experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

        # Get the SDF file path and copy it to the flattened experiment directory
        sdf_file = os.path.join(os.path.split(
            os.path.dirname(self.__exd_configuration))[0],
            experiment.environmentModel.src
        )
        shutil.copyfile(sdf_file, os.path.join(self.__temp_directory, os.path.basename(sdf_file)))

        # Update the experiment configuration file with the new path(s) and saves it
        experiment.environmentModel.src = os.path.basename(sdf_file)
        flattened_exd_configuration = os.path.join(self.__temp_directory,
                                                   os.path.basename(self.__exd_configuration))
        with open(flattened_exd_configuration, "w") as flattened_exd_configuration_file:
            flattened_exd_configuration_file.write(experiment.toDOM().toprettyxml())

        return self.__temp_directory

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
