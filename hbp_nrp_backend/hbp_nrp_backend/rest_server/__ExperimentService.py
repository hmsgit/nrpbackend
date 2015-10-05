"""
This module contains the REST implementation
for retrieving Experiment names and IDs, as well as some functions used by other
classes that retrieve experiment related files.
"""

__author__ = "Bernd Eckstein"

import os
import base64
import logging

from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, \
    NRPServicesGeneralException
from hbp_nrp_backend.exd_config.generated import exp_conf_api_gen
from hbp_nrp_cle.bibi_config.generated import bibi_api_gen
from hbp_nrp_cle.bibi_config.bibi_configuration_script import get_all_tfs

from flask import request
from pyxb import ValidationError
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication

# pylint: disable=no-self-use
# because pylint detects experiment.get_bibiConf() as no member:
# pylint: disable=maybe-no-member


LOG = logging.getLogger(__name__)


class ErrorMessages(object):
    """
    Definition of error strings
    """
    EXPERIMENT_NOT_FOUND_404 = "The experiment with the given ID was not found"
    EXPERIMENT_PREVIEW_NOT_FOUND_404 = "The experiment has no preview image"
    EXPERIMENT_BIBI_FILE_NOT_FOUND_404 = "The experiment BIBI file was not found"
    EXPERIMENT_CONF_FILE_NOT_FOUND_404 = "The experiment configuration file was not found"
    EXPERIMENT_BRAIN_FILE_NOT_FOUND_500 = "The experiment brain file was not found"
    VARIABLE_ERROR = "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
    ERROR_SAVING_FILE_500 = "Error saving file"
    ERROR_IN_BASE64_400 = "Error in base64: {0}"


@swagger.model
class ExperimentObject(object):
    """
    Swagger documentation object
    Experiment definition
    """
    resource_fields = {
        "name": fields.String(),
        "experimentConfiguration": fields.String(),
        "description": fields.String(),
        "timeout": fields.Integer(),
        "maturity": fields.String()
    }
    required = ['name', 'experimentConfiguration', 'description', 'timeout', 'maturity']


@swagger.model
@swagger.nested(exp_id_1=ExperimentObject.__name__,
                exp_id_2=ExperimentObject.__name__,
                exp_id_n=ExperimentObject.__name__)
class ExperimentDictionary(object):
    """
    Swagger documentation object
    ExperimentDictionary ... tried to make it look like a dictionary for the swagger doc
    """
    resource_fields = {
        'exp_id_1': fields.Nested(ExperimentObject.resource_fields),
        'exp_id_2': fields.Nested(ExperimentObject.resource_fields),
        'exp_id_n': fields.Nested(ExperimentObject.resource_fields)
    }


@swagger.model
@swagger.nested(data=ExperimentDictionary.__name__)
class ExperimentData(object):
    """
    Swagger documentation object
    Main Data Attribute for parsing convenience on the frontend side.
    """
    resource_fields = {
        'data': fields.Nested(ExperimentDictionary.resource_fields)
    }
    required = ['data']


class Experiment(Resource):
    """
    Implements the REST service for retrieving all stored experiments
    as a dictionary
    """

    @swagger.operation(
        notes="Gets dictionary of experiments",
        responseClass=ExperimentData.__name__,
        responseMessages=[
            {
                "code": 500,
                "message": "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
            },
            {
                "code": 200,
                "message": "Success. The experiment list was sent."
            }
        ]
    )
    def get(self):
        """
        Gets dictionary of experiments stored on the server.

        :return: Dictionary with experiments
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 200: Success. The dictionary was returned
        """

        experiments = dict(data=get_experiments())
        return experiments, 200


def get_experiments():
    """
    Read all available Experiments from disk, and add to dictionary
    :return dictionary: with string: ID, string: ExDConfig File
    """

    experiment_dir = "ExDConf"
    path = os.path.join(get_basepath(), experiment_dir)
    experiment_names = os.listdir(path)

    experiment_dict = {}

    for current in experiment_names:
        if current.lower().endswith('.xml'):
            # Parse Experiment
            experiment_conf = os.path.join(get_basepath(), experiment_dir, current)
            ex = None
            with open(experiment_conf) as exd_file:
                try:
                    ex = exp_conf_api_gen.CreateFromDocument(exd_file.read())
                except ValidationError, ve:
                    LOG.warn("Could not parse experiment configuration %s due to validation "
                             "error: %s", experiment_conf, str(ve))
                    continue

            if isinstance(ex, exp_conf_api_gen.ExD_):
                current_exp = _make_experiment(current, ex, experiment_dir)
                experiment_dict[os.path.splitext(current)[0]] = current_exp

    return experiment_dict


def _make_experiment(experiment_file, experiment, experiment_dir):
    """
    Creates and returns an dictionary with the experiment data
    :param experiment_file: filename of the XML containing the experiment
    :param experiment: the parsed experiment
    :param experiment_dir: the directory containing the XML file of the experiment
    :return dictionary with the experiment data
    """

    _timeout = experiment.timeout
    _name = experiment.name
    _description = experiment.description
    _maturity = experiment.maturity

    if _timeout is None:
        _timeout = 600
    if _name is None:
        _name = os.path.splitext(experiment_file)[0]
    if _description is None:
        _description = "No description available for this experiment."
    # Axel: maturity can be "development" or "production"
    if _maturity is None or (_maturity != "development" and _maturity != "production"):
        _maturity = "development"

    current_exp = dict(name=_name,
                       description=_description,
                       experimentConfiguration=os.path.join(experiment_dir, experiment_file),
                       timeout=_timeout,
                       maturity=_maturity)
    return current_exp


def save_file(base64_data, filename_abs, data=None):
    """
    Save a file, encoded as base64_data to specified filename_abs
    :param base64_data:  base64 encoded data
    :param filename_abs: the absolute filename
    :param data: use instead of base64_data, when you wand to write a string as-is to the disk
    :return filename (string), when file is saved
    :exception NRPServicesClientErrorException, errorNo: 401: Error in base64
    """

    if data is None:
        try:
            data = base64.decodestring(base64_data)
        except Exception as _ex:
            raise NRPServicesClientErrorException(
                ErrorMessages.ERROR_IN_BASE64_400.format(_ex.message)
            )

    base_path = os.path.abspath(filename_abs)
    allowed_path = get_basepath()

    if base_path.startswith(allowed_path):
        pass
    else:
        LOG.error("Path not accepted: '{0}' is not a subdirectory of allowed path: '{1}'."
                  .format(base_path, allowed_path))
        raise NRPServicesGeneralException(ErrorMessages.ERROR_SAVING_FILE_500,
                                          "Server Error: {0}, {1}".format(base_path,
                                                                          allowed_path), 500)

    filename = "{0}_new".format(filename_abs)
    with open(filename, 'wb') as _file:
        _file.write(data)
        return filename


def get_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise NRPServicesGeneralException(ErrorMessages.VARIABLE_ERROR, "Server Error", 500)

    return path


def get_experiment_rel(exp_id):
    """
    :return: relative name of the experiment xml
    """

    experiment_dict = get_experiments()
    if exp_id not in experiment_dict:
        raise NRPServicesClientErrorException(
            ErrorMessages.EXPERIMENT_NOT_FOUND_404,
            error_code=404
        )

    # Gets Experiments relative filename
    experiment_file = experiment_dict[exp_id]['experimentConfiguration']
    return experiment_file


def get_experiment_conf(exp_id):
    """
    Gets the filename of the conf file

    :param exp_id: id of the experiment
    :return Absolute path to experiment xml file
    """

    experiment_file = get_experiment_rel(exp_id)
    experiment_conf = os.path.join(get_basepath(), experiment_file)
    return experiment_conf


def get_bibi_file(exp_id):
    """
    Gets the filename of the bibi file

    :param exp_id: id of the experiment
    :return Absolute path to experiment bibi file
    """
    experiment_file = get_experiment_conf(exp_id)

    # parse experiment configuration
    with open(experiment_file) as exd_file:
        experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
    assert isinstance(experiment, exp_conf_api_gen.ExD_)
    bibi_file = experiment.bibiConf.src
    bibi_conf = os.path.join(get_basepath(), bibi_file)
    return bibi_conf


def get_brain_file(exp_id):
    """
    Gets the brain filename

    :param exp_id: id of the experiment
    :return: Absolute path to experiment brain file
    """
    bibi_file_name = get_bibi_file(exp_id)
    with open(bibi_file_name) as bibi_file:
        bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        assert isinstance(bibi, bibi_api_gen.BIBIConfiguration)
        brain_file_name = bibi.brainModel.file
        return os.path.join(get_basepath(), brain_file_name)

    return None


def get_control_state_machine_files(exp_id):
    """
    Gets the control state machine file names

    :param exp_id: id of the experiment
    :return Dict with name, Absolute path to state machine files
    """
    return _get_state_machine_files(exp_id, 'control')


def get_evaluation_state_machine_files(exp_id):
    """
    Gets the evaluation state machine file names

    :param exp_id: id of the experiment
    :return Dict with name, Absolute path to state machine files
    """
    return _get_state_machine_files(exp_id, 'evaluation')


def _get_state_machine_files(exp_id, which):
    """
    Gets the state machine file names

    :param exp_id: id of the experiment
    :param string which: 'control' or 'evaluation'
    :return: Dict with name, Absolute path to state machine files
    """
    ret = dict()
    experiment_file = get_experiment_conf(exp_id)
    # parse experiment configuration
    with open(experiment_file) as exd_file:
        experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

    assert isinstance(experiment, exp_conf_api_gen.ExD_)

    if which is 'control':
        files = experiment.experimentControl
    elif which is 'evaluation':
        files = experiment.experimentEvaluation

    if files:
        for sm in files.stateMachine:
            if isinstance(sm, exp_conf_api_gen.SMACHStateMachine):
                ret[sm.id] = os.path.join(get_basepath(), sm.src)

    return ret


def get_username():
    """
    Gets the name of the current user

    :return: string: username
    """
    user_name = UserAuthentication.get_x_user_name_header(request)
    return user_name


def substitute_bibi_transferfunctions(bibi_file, tf_list):
    """
    Create a new bibi, containing given transfer functions

    :param bibi_file: filename of bibi
    :param tf_list: list(string) of transfer functions (python)
    :return: the new bibi as string containing these transfer functions
    """

    with open(bibi_file) as bibi_xml:
        bibi = bibi_api_gen.CreateFromDocument(bibi_xml.read())
    assert isinstance(bibi, bibi_api_gen.BIBIConfiguration)

    # Remove all transfer functions from BIBI
    del bibi.transferFunction[:]

    # Add given TFs
    for current in tf_list:
        ptf = bibi_api_gen.PythonTransferFunction()
        ptf.append(current)
        bibi.transferFunction.append(ptf)

    return str(bibi.toxml("utf-8"))


def get_transfer_functions(bibi_file):
    """
    Generates cle from bibi_file and returns the transfer functions as array of strings

    :param bibi_file: bibi filename
    :type bibi_file: basestring
    :return: array of strings containing transfer functions
    """

    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    return get_all_tfs(bibi_file, models_path)
