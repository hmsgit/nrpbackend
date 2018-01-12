# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
Wrapper around the storage API
"""

import os
import logging
import tempfile
import requests
import shutil
from hbp_nrp_backend.config import Config
from pyxb import ValidationError
from xml.sax import SAXParseException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_commons.generated import exp_conf_api_gen, environment_conf_api_gen, bibi_api_gen, \
    robot_conf_api_gen
from hbp_nrp_backend.rest_server.__ExperimentService import get_experiment_basepath

__author__ = "Manos Angelidis"
logger = logging.getLogger(__name__)


class StorageClient(object):
    """
    Wrapper around the storage server API. Users of this class should first
    call the authentication function to retrieve a token, before making the
    requests to the storage server
    """

    def __init__(self):
        """
        Creates the storage client
        """

        def make_temp_directory():
            """
            Returns a temp directory where all the simulation related files are
            stored. This directory starts with nrpTemp

            :return the path to the temporary directory
            """
            for entry in os.listdir(tempfile.gettempdir()):
                if entry.startswith('nrpTemp'):
                    return os.path.join(tempfile.gettempdir(), entry)
            return None
        self.__proxy_url = Config.LOCAL_STORAGE_URI['storage_uri']
        if not make_temp_directory():
            self.__temp_directory = tempfile.mkdtemp(
                prefix="nrpTemp", suffix="")
        else:
            self.__temp_directory = make_temp_directory()

    def get_temp_directory(self):
        """
        Returns the temporary directory where all the simulation based files are stored
        """
        return self.__temp_directory

    # Wrappers around the storage API
    def authenticate(self, user, password):
        """
        Authenticates the user based on the user name and password
        :param user: the username of the user
        :param password: the password of the user
        :return: the token
        """
        try:
            res = requests.post(self.__proxy_url +
                                '/authentication/authenticate',
                                json={"user": user, "password": password})
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def list_experiments(self, token, context_id, get_all=False, name=None):
        """
        Lists the experiments the user has access to depending on his token
        :param token: a valid token to be used for the request
        :param context_id: the context_id if we are using collab storage
        :param get_all: a parameter to return all the available experiments,
                        not only the ones available to a specific user
        :param name: if we want to get a specific folder i.e. the robots
        :return: an array of all the available to the user experiments
        """
        headers = {'Authorization': 'Bearer ' +
                   token, 'context-id': context_id}

        try:
            url = self.__proxy_url + '/storage/experiments'
            if get_all and name:
                url += '?all=true&filter=' + name
            elif name:
                url += '?filter=' + name
            elif get_all:
                url += '?all=true'
            res = requests.get(url, headers=headers)

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def create_experiment(self, token, experiment, context_id):
        """
        Creates a new experiment based on the experiment name and the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param context_id: the context_id of the experiment
        :return: if successful, the name of the experiment
        """
        try:
            headers = {'Authorization': 'Bearer ' + token,
                       'context-id': context_id}
            res = requests.put(self.__proxy_url +
                               '/storage/{0}'.format(experiment), headers=headers)
            # for some weird reason when the experiment exists we get a 418
            # status code!
            if res.status_code < 200 or (res.status_code >= 300 and not res.status_code == 418):
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                if res.text == 'Experiment already exists':
                    logger.info(res.json())
                else:
                    return res.json()['uuid']
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_file(self, token, experiment, filename, byname=False, zipped=False):
        """
        Gets a file under an experiment based on the filename
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to return
        :return: if successful, the content of the file
        """
        try:
            headers = {'Authorization': 'Bearer ' + token}
            if not byname:
                res = requests.get(self.__proxy_url +
                                   '/storage/{0}/{1}'.format(
                                       experiment, filename), headers=headers)
            else:
                res = requests.get(self.__proxy_url +
                                   '/storage/{0}/{1}?byname=true'.format(
                                       experiment, filename), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            if zipped:
                return res.content
            else:
                return res.text
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def delete_file(self, token, experiment, filename):
        """
        Deletes a file under under an experiment based on the
        experiment name and the filename. Needs the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to delee
        :return: if successful, the name of the deleted file
        """
        headers = {'Authorization': 'Bearer ' + token}
        try:
            res = requests.delete(self.__proxy_url +
                                  '/storage/{0}/{1}'.format(
                                      experiment, filename), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return "Success"
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def create_or_update(self, token, experiment, filename, content, content_type):
        """
        Creates or updates a file under an experiment
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to update/create
        :param content: the content of the file
        :param content_type: the content type of the file i.e. text/plain or
        aplication/octet-stream
        """
        try:
            headers = {'content-type': content_type,
                       'Authorization': 'Bearer ' + token}
            res = requests.post(self.__proxy_url +
                                '/storage/{0}/{1}'.format(
                                    experiment, filename),
                                headers=headers, data=content)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return res.status_code
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def create_folder(self, token, experiment, name):
        """
        Creates a folder under an experiment. If the folder exists we reuse it
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param name: the name of the folder to create
        """
        try:
            headers = {'Authorization': 'Bearer ' + token}
            res = requests.post(self.__proxy_url +
                                '/storage/{0}/{1}{2}'.format(
                                    experiment, name, '?type=folder'),
                                headers=headers)
            if res.status_code == 400:
                logger.info('The folder with the name {0} already exists in the storage, \
                reusing'.format(name))
                return 200
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def list_files(self, token, experiment):
        """
        Lists all the files under an experiment based on the
        experiment name and the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :return: if successful, the files under the experiment
        """
        try:
            headers = {
                'Authorization': 'Bearer ' + token,
            }
            res = requests.get(self.__proxy_url +
                               '/storage/{0}'.format(
                                   experiment), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                results = []
                # we have to filter out the folders
                for entry in res.json():
                    if entry['type'] == 'file':
                        results.append(entry)
                return results
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    # HELPER FUNCTIONS
    def clone_file(self, filename, token, experiment):
        """
        Clones a file according to a given filename to a temporary folder.
        The caller has then the responsibility of managing this folder.

        :param filename: The filename of the file to clone
        :param token: The token of the request
        :param filename: The experiment which contains the file
        :return: The local path of the cloned file,
        """
        found = False
        for folder_entry in self.list_files(token, experiment):
            if filename in folder_entry['name']:
                found = True
        if found:
            clone_destination = os.path.join(self.__temp_directory, filename)
            with open(clone_destination, "w") as f:
                f.write(self.get_file(
                    token, experiment, filename, byname=True))
            return clone_destination
        else:
            return None

    def clone_all_experiment_files(self, token, experiment, new_folder=False):
        """
        Clones all the experiment files to a temporary folder.
        The caller has then the responsibility of managing this folder.

        :param token: The token of the request
        :param experiment: The experiment to clone
        :param new_folder: specifies whether we want to clone the files again
        or use the existing temporary nrpTemp folder. Used in reset
        :return: A dictionary containing the paths to the experiment files
        as well as the path to the temporary folder
        """
        experiment_paths = dict()
        list_files_to_clone = self.list_files(token, experiment)
        if new_folder:
            destination_directory = tempfile.mkdtemp()
        else:
            destination_directory = self.get_temp_directory()
        for file_under_experiment in list_files_to_clone:
            file_clone_destination = os.path.join(
                destination_directory, file_under_experiment['name'])
            with open(file_clone_destination, "w") as f:
                file_contents = self.get_file(
                    token, experiment, file_under_experiment['name'], byname=True)

                # in order to return the environment and experiment path we have
                # to read the .exc
                if 'experiment_configuration.exc' in str(file_clone_destination):
                    experiment_paths[
                        'experiment_conf'] = file_clone_destination
                    env_filename = exp_conf_api_gen.CreateFromDocument(
                        file_contents).environmentModel.src
                    experiment_paths['environment_conf'] = os.path.join(
                        destination_directory, env_filename)

                f.write(file_contents)
        return destination_directory, experiment_paths

    @staticmethod
    def parse_and_check_file_is_valid(filepath, create_obj_function, instance_type):
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
                        "%s configuration file content is not valid." % (
                            filepath),
                        "File not valid"
                    )
            except (ValidationError, SAXParseException) as ve:
                raise NRPServicesGeneralException(
                    "Could not parse file %s due to validation error: %s" % (
                        filepath, str(ve)),
                    "File not valid"
                )
        return file_obj

    def create_unique_experiment_id(self, basename, suffix, experiments_names):
        """
        Creates a unique experiment_id which does not exist in the user's
        experiments list. If an experiment with the same basename exists
        a numerical suffix is appended. For example if there exist experiments
        with name 'User Experiment_i' for i = 1 to n - 1, return 'User Experiment_n'.
        Works also if a random experiment in the range i = 1 to n - 1 is deleted
        Recursive functions
        :param basename: The name of the experiment to create. I.e. 'Experiment'
        :param suffix: A numerical value to append
        :param experiments_names: The names of the experiments accessible to the user
        :return: A unique experiment folder id
        """
        if (basename + '_' + str(suffix)) in experiments_names:
            suffix += 1
            return self.create_unique_experiment_id(basename, suffix, experiments_names)
        else:
            return (basename + '_' + str(suffix))

    def clone_experiment_template_to_storage(self,
                                             token,
                                             exp_configuration,
                                             paths=None,
                                             context_id=None):
        """
        Takes an experiment template and clones it into the storage. This creates
        a new experiment folder for the specific user
        :param token: The token of the user
        :param exp_configuration: The experiment configuration file of the template to clone
        :return: The UUID of the created folder
        """

        list_experiments = self.list_experiments(
            token, context_id, get_all=True)

        experiment_folder_uuid = self.create_experiment(
            token, self.create_unique_experiment_id(
                'Experiment', 0, [exp['name'] for exp in list_experiments]),
            context_id)

        with _FlattenedExperimentDirectory(exp_configuration, paths) as temporary_folder:
            logger.debug(
                "Copying the flattened experiment files to the storage")
            for filename in os.listdir(temporary_folder):
                filepath = os.path.join(temporary_folder, filename)
                _, ext = os.path.splitext(filepath)
                mimetype = 'application/octet-stream' if ext in {'.png', '.jpg'} else 'text/plain'
                with open(filepath) as temp_image:
                    self.create_or_update(token,
                                          experiment_folder_uuid,
                                          filename,
                                          temp_image,
                                          mimetype)
        return experiment_folder_uuid

    def get_folder_uuid_by_name(self, token, context_id, folder_name):
        """
        Returns the uuid of a folder provided its name
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param folder_name: the name of the folder
        :return: if found, the uuid of the named folder
        """
        folders = self.list_experiments(
            token, context_id, get_all=True, name=folder_name)
        for folder in folders:
            if folder["name"] == folder_name:
                return folder["uuid"]

    # pylint: disable=no-self-use
    def remove_temp_directory(self):
        """
        Removes the temporary directory where all the simulation based files are stored
        """
        for entry in os.listdir(tempfile.gettempdir()):
            if entry.startswith('nrpTemp'):
                nrp_temp = os.path.join(tempfile.gettempdir(), entry)
                logger.debug(
                    "removing the temporary configuration folder %s",
                    nrp_temp
                )
                shutil.rmtree(nrp_temp)


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

    def __init__(self, exp_configuration, models_paths):
        """
        Constructor. Does nothing more that saving the exp_configuration parameter.
        :param exp_configuration: The experiment configuration file of the template to clone.
        """
        self.__temp_directory = None
        self.__models_paths = models_paths
        # exp_configuration may look like:
        # $NRP_EXPERIMENTS_DIRECTORY/braitenberg_husky/ExDBraitenbergHuskySBC.xml
        # We extract: $NRP_EXPERIMENTS_DIRECTORY
        self.__exp_configuration = os.path.join(
            get_experiment_basepath(), exp_configuration)
        self.__experiment_folder = os.path.dirname(self.__exp_configuration)
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
                shutil.copyfile(sm_file, os.path.join(
                    self.__temp_directory, sm.src))
        # Get the experiment image and copy it into the flattened experiment
        # directory
        img_file = os.path.join(self.__experiment_folder,
                                experiment_dom.thumbnail)
        shutil.copyfile(img_file, os.path.join(
            self.__temp_directory, os.path.basename(img_file)))
        # Update the experiment thumbnail file with the new path
        experiment_dom.thumbnail = os.path.basename(img_file)
        # Get the SDF file path and copy it into the flattened experiment directory
        # if we are creating a new experiment we have to change the path to the
        # environment
        if self.__models_paths is not None and self.__models_paths['envPath'] is not None:
            sdf_file = \
                self._parse_models_config_file(
                    self.__models_paths['envPath'], 'environments', environment_conf_api_gen)
        else:
            sdf_file = os.path.join(
                self.__models_folder, experiment_dom.environmentModel.src)
        shutil.copyfile(sdf_file, os.path.join(
            self.__temp_directory, os.path.basename(sdf_file)))
        # Update the experiment configuration file with the new paths
        experiment_dom.environmentModel.src = os.path.basename(sdf_file)
        # Get the BIBI file path and copy it into the flattened experiment
        # directory
        bibi_configuration_file = os.path.join(
            self.__experiment_folder, experiment_dom.bibiConf.src
        )
        # Update the BIBI configuration file with the new paths
        experiment_dom.bibiConf.src = 'bibi_configuration.bibi'
        flattened_exp_configuration_file = \
            os.path.join(
                self.__temp_directory,
                'experiment_configuration.exc'
            )
        # save experiment configuration file
        with open(flattened_exp_configuration_file, "w") as f:
            f.write(experiment_dom.toDOM().toprettyxml())

        # copy experiment configuration file
        for conf in experiment_dom.configuration:
            self.__copy_config_file(conf.src)

        # copy launch file
        if experiment_dom.rosLaunch:
            self.__copy_config_file(experiment_dom.rosLaunch.src)

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
        shutil.copyfile(config_filepath, os.path.join(
            self.__temp_directory, config_filename))

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
            logger.debug("Copying config file {} to storage.".format(conf.src))
            self.__copy_config_file(conf.src)

        # Get the robot file path and copy it into the flattened experiment
        # directory
        if self.__models_paths is not None and self.__models_paths['robotPath'] is not None:
            robot_sdf_file = \
                self._parse_models_config_file(
                    self.__models_paths['robotPath'], 'robots', robot_conf_api_gen)
        else:
            robot_sdf_file = os.path.join(
                self.__models_folder, bibi_configuration_dom.bodyModel)
        robot_sdf_file_name = os.path.basename(robot_sdf_file)
        shutil.copyfile(robot_sdf_file, os.path.join(
            self.__temp_directory, robot_sdf_file_name))
        bibi_configuration_dom.bodyModel = bibi_api_gen.SDFFilename(
            robot_sdf_file_name)

        robot_config = os.path.join(os.path.dirname(robot_sdf_file), 'model.config')
        shutil.copyfile(robot_config, os.path.join(
            self.__temp_directory, 'robot.config'))

        # Get the PyNN file path and copy it into the flattened experiment
        # directory
        if self.__models_paths is not None and self.__models_paths['brainPath'] is not None:
            brain_file = \
                os.path.join(self.__models_folder, 'brains',
                             self.__models_paths['brainPath'])
        else:
            brain_file = os.path.join(
                self.__models_folder, bibi_configuration_dom.brainModel.file)
        brain_file_name = os.path.basename(brain_file)
        shutil.copyfile(brain_file, os.path.join(
            self.__temp_directory, brain_file_name))
        bibi_configuration_dom.brainModel.file = bibi_api_gen.PythonFilename(
            brain_file_name)

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
            'bibi_configuration.bibi'
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
        if not (exc_type or exc_value or traceback):
            logger.debug("Clean up temporary directory " +
                         self.__temp_directory)
            shutil.rmtree(self.__temp_directory)

    def _parse_models_config_file(self, model_config_path, model_type, model_api_gen):
        """
        Parse the model.config files from the paths we receive from
        the frontend. These files contain the path to the models
        (environment and robot)
        :param model_config_path: The path to the model we are getting
        from the frontend. Looks like: /lauron_model/model.sdf
        :param model_type: Differentiate between environment, robot
        :return model_sdf_file: The absolute path to the model
        """
        # Looks like : $NRR/Models/robots/lauron_model/model.config
        model_config_file = \
            os.path.join(self.__models_folder, model_type, model_config_path)
        with open(model_config_file) as model_config:
            model_config_dom = \
                model_api_gen.CreateFromDocument(model_config.read())
            # we read the relative path to the sdf
            model_sdf_value = model_config_dom.sdf.value()
            # we extract the name of the model folder, i.e. lauron_model
            model_folder_name = os.path.split(model_config_path)[0]
            # the final path looks like
            # $NRR/Models/robots/lauron_model/model.sdf
            model_sdf_file = \
                os.path.join(self.__models_folder,
                             model_type,
                             model_folder_name,
                             model_sdf_value)
            return model_sdf_file
