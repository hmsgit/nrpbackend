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
import urllib
import re
from hbp_nrp_backend.config import Config
from pyxb import ValidationError
from xml.sax import SAXParseException
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
from hbp_nrp_commons.generated import exp_conf_api_gen

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
        self.__proxy_url = Config.LOCAL_STORAGE_URI['storage_uri']
        simdir = os.environ['NRP_SIMULATION_DIR']
        if simdir is None:
            raise Exception("Server Error. NRP_SIMULATION_DIR not defined.")

        if not os.path.exists(simdir):
            try:
                os.mkdir(simdir)
            except OSError as e:
                raise e
        self.__simulation_directory = simdir

        # Paths related to the textures

        # The path to the gazebo media path
        self.__local_gazebo_path = os.path.join(
            os.environ['HOME'], '.local', 'share', 'gazebo-7', 'media')

        # The path to the gzweb media path
        self.__gzweb_assets_media_path = os.path.join(
            os.environ['HBP'], 'gzweb', 'http', 'client', 'assets', 'media')

        # The path to the gzweb custom_textures path
        self.__gzweb_custom_textures_path = os.path.join(
            os.environ['HBP'], 'gzweb', 'http', 'client', 'assets', 'custom_textures')

        # Array that holds all the textures related paths
        self.__texture_directories = [
            self.__local_gazebo_path,
            self.__gzweb_assets_media_path,
            self.__gzweb_custom_textures_path]

        # adding the resources folder into the created temp_directory
        self.__resources_path = os.path.join(
            self.__simulation_directory, "resources")

        # folders in resources we want to filter
        self.__filtered_resources = ['textures']

    def get_simulation_directory(self):
        """
        Returns the simulation directory where all the simulation based files are stored
        """
        return self.__simulation_directory

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

    def get_file(self, token, experiment, filename, byname=False, zipped=False, is_texture=False):
        """
        Gets a file under an experiment based on the filename and on the filetype
        Depending on the file type we either:
            return the .content of the response if it is a zip
            return the .raw of the response if it is an image
            return the .text if it is a text file
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to return
        :param zipped: flag denoting that "filename" is a zip file
        :param is_texture: flag denoting that "filename" is a texture file
        :return: if successful, the content of the file
        """
        try:
            headers = {'Authorization': 'Bearer ' + token}
            if not byname:
                res = requests.get(self.__proxy_url +
                                   '/storage/{0}/{1}'.format(
                                       experiment, filename), headers=headers, stream=is_texture)
            else:
                res = requests.get(self.__proxy_url +
                                   '/storage/{0}/{1}?byname=true'.format(
                                       experiment, filename), headers=headers, stream=is_texture)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            if zipped:
                return res.content
            if is_texture:
                res.raw.decode_content = True
                return res.raw
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
        :param filename: the name of the file to delete
        :return: if successful, the name of the deleted file
        """
        headers = {'Authorization': 'Bearer ' + token}
        try:
            path = urllib.quote(os.path.join(experiment, filename), safe='')
            res = requests.delete(self.__proxy_url + '/storage/{path}'.format(path=path),
                                  headers=headers)
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
                             application/octet-stream
        """
        try:
            headers = {'content-type': content_type,
                       'Authorization': 'Bearer ' + token}
            res = requests.post(self.__proxy_url + '/storage/{0}/{1}'.format(experiment, filename),
                                headers=headers, data=content)

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception('Failed to communicate with the storage server, status code ' +
                                str(res.status_code))
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

    def list_files(self, token, experiment, folder=None):
        """
        Lists all the files under an experiment based on the
        experiment name and the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param folder: it is a boolean variable that indicates if list_files returns folders or not.

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
                # folder variable is added because copy_resources_folders_to_tmp needs
                #  to list the folders too
                for entry in res.json():
                    if (entry['type'] == 'file' or folder) \
                            and not self.check_file_extension(entry['name'], ['.swp']):
                        results.append(entry)
                return results
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_custom_models(self, token, context_id, folder_name):
        """
        Returns the contents of a custom models folder provided its name
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param folder_name: the name of the folder
        :return: if found, the uuid of the named folder
        """
        try:
            headers = {
                'Authorization': 'Bearer ' + token,
                'context-id': context_id
            }
            res = requests.get(self.__proxy_url +
                               '/storage/custommodels/{0}'.format(
                                   folder_name), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_custom_model(self, token, context_id, model_path):
        """
        Returns a custom model provided its path
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param folder_name: the name of the folder
        :return: if found, the uuid of the named folder
        """
        try:
            headers = {
                'Authorization': 'Bearer ' + token,
                'context-id': context_id
            }
            res = requests.get(self.__proxy_url +
                               '/storage/custommodel/{0}'.format(
                                   model_path), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return res.content
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_textures(self, experiment, token):
        """
        Returns the contents of the resources/textures experiment folder
        :param experiment: the name of the experiment
        :param token: a valid token to be used for the request
        :return: if found, the list of textures
        """
        try:
            headers = {
                'Authorization': 'Bearer ' + token
            }
            textures_path = urllib.quote_plus('/resources/textures')
            res = requests.get(self.__proxy_url +
                               '/storage/{0}{1}'.format(experiment, textures_path), headers=headers)
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code '
                    + str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    # HELPER FUNCTIONS
    def clone_file(self, filename, token, experiment):
        """
        Clones a file according to a given filename to a simulation folder.
        The caller then has the responsibility of managing this folder.

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
            clone_destination = os.path.join(
                self.__simulation_directory, filename)
            with open(clone_destination, "w") as f:
                f.write(self.get_file(
                    token, experiment, filename, byname=True))
            return clone_destination
        else:
            return None

    def copy_file_content(self, token, src_folder, dest_folder, filename, is_texture=False):
        """
        copy the content of file located in the Storage into the proper tmp folder

        :param token: The token of the request
        :param src_folder: folder location where it will be copy from
        :param dest_folder: folder location where it will be copy to
        :param filename: name of the file to be copied.
        :param is_texture: flag to signal a texture.
        """
        _, ext = os.path.splitext(filename)
        # if the file is a texture we have to copy a file object, not a string
        if is_texture:
            with open(os.path.join(src_folder, filename), "wb") as f:
                shutil.copyfileobj(
                    self.get_file(
                        token,
                        dest_folder, filename,
                        byname=True, zipped=False, is_texture=is_texture
                    ), f
                )
        else:
            with open(os.path.join(src_folder, filename), "w") as f:
                f.write(
                    self.get_file(
                        token,
                        dest_folder, filename,
                        byname=True, zipped=(ext.lower() == ".zip"), is_texture=is_texture
                    )
                )

    # pylint: disable=no-self-use
    def check_file_extension(self, filename, extensions):
        """
        checks if an file is of a certain extension

        :param filename: the file name
        :param extensions: the extensions list to check
        """
        return os.path.splitext(filename)[1].lower() in extensions

    # pylint: disable=no-self-use
    def check_create_folder(self, folder_path):
        """
        checks if the folder exist and if it does not exist, then it is created it

        :param folder_path: folder location to be checked
        """
        try:
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
            return None
        except OSError as ex:
            logger.exception(
                'An error happened trying to create ' + folder_path)
            raise ex

    def copy_folder_content_to_tmp(self, token, folder):
        """
        copy the content of the folder located in storage/experiment into tmp folder

        :param token: The token of the request
        :param folder: the folder in the storage folder to copy in tmp folder,
                       it has included the uuid of the experiment
        """
        folder['fullpath'] = folder['name']
        child_folders = [folder]
        folder_path = ''
        while child_folders:
            actual_folder = child_folders.pop()
            folder_path = actual_folder['fullpath']
            folder_uuid = urllib.quote_plus(actual_folder['uuid'])
            for folder_entry in self.list_files(token, folder_uuid, True):
                if folder_entry['type'] == 'folder' and folder_entry['name'] \
                        not in self.__filtered_resources:
                    folder_entry['fullpath'] = os.path.join(folder_path, folder_entry['name'])
                    child_folders.append(folder_entry)
                if folder_entry['type'] == 'file':
                    folder_tmp_path = str(os.path.join(
                        self.__simulation_directory, folder_path))
                    self.check_create_folder(folder_tmp_path)
                    self.copy_file_content(
                        token, folder_tmp_path, folder_uuid, folder_entry['name'])

    # pylint: disable=no-self-use
    def delete_directory_content(self, folder_path):
        """
        delete the content of the folder.

        :param folder_path: The path of the folder.
        """
        for file_entry in os.listdir(folder_path):

            file_path = os.path.join(folder_path, file_entry)
            if os.path.isfile(file_path):
                os.unlink(file_path)
        return None

    def copy_resources_folders_to_tmp(self, token, experiment):
        """
        Copy the resources folder located in storage/experiment into tmp folder

        :param token: The token of the request
        :param experiment: The experiment which contains the resource folder
        """
        try:
            for folder_entry in self.list_files(token, experiment, True):
                if folder_entry['name'] == 'resources' and folder_entry['type'] == 'folder':
                    if os.path.exists(self.__resources_path):
                        self.delete_directory_content(self.__resources_path)
                    self.copy_folder_content_to_tmp(
                        token, folder_entry)
        except Exception as ex:
            logger.exception(
                'An error happened trying to copy resources to tmp ')
            raise ex

    def create_material_from_textures(self, textures):
        """
        Algorithm to create a new material from the textures. For every texture
        we append a new default material which points to the texture.
        """
        material_script = ''
        for texture in textures:
            # The material script is a stripped down version of an OGRE material script
            # see http://wiki.ogre3d.org/Materials for the documentation
            material = 'material ' + texture['name'] + '\n' \
                + ' { ' + '\n' \
                + '   technique' + '\n' \
                + '    {' + '\n' \
                + '      pass' + '\n' \
                + '      {' + '\n' \
                + '          ambient 1 1 1 1.000000' + '\n' \
                + '          diffuse 1 1 1 1.000000' + '\n' \
                + '          specular 0.03 0.03 0.03 1.000000' + '\n' \
                + '          texture_unit' + '\n' \
                + '          {' + '\n' \
                + '              texture ' + texture['name'] + '\n' \
                + '          }' + '\n' \
                + '      }' + '\n' \
                + '    }' + '\n' \
                + ' }' + '\n'
            material_script += material

        # Create a custom.material script and pass it to all the required directories
        for directory in self.__texture_directories:
            with open(os.path.join(directory, 'materials', 'scripts', 'custom.material'), 'w') as f:
                f.write(material_script)

    def filter_textures(self, textures):
        """
        Returns only the textures from /resources/textures
        """
        # Regexp checks if the texture is a png, gif, or jpeg
        return [texture for texture in textures if re.search(r"\.(jpe?g|png|gif)$", texture["name"],
                                                             re.IGNORECASE) is not None]

    def create_textures_paths(self):
        """
        Generate the folder structure under assets. Looks like:
        relative_path/materials/scripts/custom.material
        relative_path/materials/textures/texture1...n
        """
        relative_path = ''
        for directory in self.__texture_directories:
            for path in ['materials']:
                self.check_create_folder(os.path.join(
                    directory, relative_path))
                relative_path = os.path.join(relative_path, path)
            self.check_create_folder(os.path.join(
                directory, relative_path, 'scripts'))
            self.check_create_folder(os.path.join(
                directory, relative_path, 'textures'))
            relative_path = ''

    def generate_textures(self, experiment, token):
        """
        Clones all the contents of the textures folders to the temporary directory and
        creates the structure required by gazebo and gzweb.

        :param token: The token of the request
        :param experiment: The experiment which contains the textures folder
        """
        # Get the contents of the resouces/textures folder from the proxy and filter them
        # to get only the images
        textures = self.filter_textures(self.get_textures(experiment, token))

        # If we find images we proceed to create paths in the backend
        if textures:

            # Create the paths in ~/.local/share/gazebo-7/media
            # $NRP/gzweb/http/client/assets/media | custom_textures
            self.create_textures_paths()

            # Generate a custom material file which points to all the textures
            self.create_material_from_textures(textures)

            # For the backend directories and for every texture copy over the textures
            for directory in self.__texture_directories:
                for texture in textures:
                    self.copy_file_content(token, os.path.join(directory, 'materials',
                                                               'textures'),
                                           urllib.quote_plus(
                                               experiment + '/resources/textures'),
                                           texture['name'], is_texture=True)
    # pylint: disable=broad-except

    def clone_all_experiment_files(self, token, experiment, new_folder=False):
        """
        Clones all the experiment files to a simulation folder.
        The caller has then the responsibility of managing this folder.

        :param token: The token of the request
        :param experiment: The experiment to clone
        :param new_folder: specifies whether we want to clone the files again
        or use the existing simulation 'simdir' folder. Used in reset
        :return: A dictionary containing the paths to the experiment files
        as well as the path to the simulation folder
        """
        experiment_paths = dict()
        list_entries_to_clone = self.list_files(token, experiment, folder=True)
        self.copy_resources_folders_to_tmp(token, experiment)

        # if something goes wrong while generating textures we just log the error
        # and continue like nothing happened
        try:
            self.generate_textures(experiment, token)
        except Exception as e:
            logger.info(
                "Could not generate textures, error occured : %s", (str(e)))

        if new_folder:
            destination_directory = tempfile.mkdtemp()
        else:
            destination_directory = self.get_simulation_directory()

        for entry_to_clone in list_entries_to_clone:
            if entry_to_clone['type'] == 'folder':
                self.copy_folder_content_to_tmp(token, entry_to_clone)
            else:  # file
                file_clone_destination = os.path.join(
                    destination_directory, entry_to_clone['name'])
                with open(file_clone_destination, "w") as f:

                    zipped = os.path.splitext(entry_to_clone['name'])[
                        1].lower() == '.zip'

                    file_contents = self.get_file(
                        token, experiment, entry_to_clone['name'],
                        byname=True, zipped=zipped)
                    # in order to return the environment and experiment path
                    # we have to read the .exc
                    if 'experiment_configuration.exc' in str(file_clone_destination):
                        experiment_paths['experiment_conf'] = file_clone_destination
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
        Removes the simulation directory where all the simulation based files are stored
        """
        logger.debug(
            "removing the simulation configuration folder %s",
            self.__simulation_directory
        )
        shutil.rmtree(self.__simulation_directory)


def get_model_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_PATHS'
    """
    paths = os.environ.get('NRP_MODELS_PATHS')
    if paths is None:
        raise Exception("Server Error. NRP_MODELS_PATHS not defined.")

    models_dirs = [x for x in paths.split(':') if os.path.isdir(x)]

    return models_dirs


def find_file_in_paths(filename, paths):
    """
    :return: returns the absolute path of the first file found path lists.
             if not found returns and empty string.
    """
    logger.info("Finding file {0} in paths {1}".format(filename, paths))

    for path in paths:
        if os.path.isfile(os.path.join(path, filename)):
            return os.path.join(path, filename)
    return None
