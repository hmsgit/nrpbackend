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
This module contains helper functionality to convert a CLE Transfer Function into a structured
format used to pass transfer functions via ROS
"""

from cle_ros_msgs.msg import ExperimentPopulationInfo
from hbp_nrp_cleserver.bibi_config.TransferFunctionASTParser import TransferFunctionASTParser, \
    StructureType
import logging
import json

__author__ = "Georg Hinkel"

device_type_infos = {
    "ACSource": ("nrp.ac_source", False),
    "DCSource": ("nrp.dc_source", False),
    "FixedFrequency": ("nrp.fixed_frequency", False),
    "LeakyIntegratorAlpha": ("nrp.leaky_integrator_alpha", True),
    "LeakyIntegratorExp": ("nrp.leaky_integrator_exp", True),
    "NCSource": ("nrp.nc_source", False),
    "Poisson": ("nrp.poisson", False),
    "SpikeRecorder": ("nrp.spike_recorder", True),
    "PopulationRate": ("nrp.population_rate", True)
}


logger = logging.getLogger(__name__)
indentation = ' ' * 4


def indent(code):
    """
    Indents the given code with four spaces

    :param code: The given code
    :return: An indented string
    """
    return indentation + ('\n' + indentation).join(code.split('\n'))


def _generate_neurons(neurons):
    """
    Generates the code for a structured neuron information

    :param neurons: The population info
    :return: The code to be generated
    """
    if neurons.type == ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION:
        return "nrp.brain.{0}".format(neurons.name)
    elif neurons.type == ExperimentPopulationInfo.TYPE_POPULATION_SLICE:
        if neurons.stop == neurons.start + 1:
            return "nrp.brain.{0}[{1}]".format(neurons.name, neurons.start)
        else:
            return "nrp.brain.{0}[slice({1},{2},{3})]"\
                .format(neurons.name, neurons.start, neurons.stop, neurons.step)
    elif neurons.type == ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW:
        return "nrp.brain.{0}[[{1}]]".format(neurons.name, ", ".join(neurons.ids))


def __generate_device(device):
    """
    Generates the code for a neuron device

    :param device:
    :return:
    """
    info = device_type_infos[device.type]
    if info[1]:  # device is spike sink
        return '@nrp.MapSpikeSink("{0}", {1}, {2})\n'\
            .format(device.name, _generate_neurons(device.neurons), info[0])
    else:
        return '@nrp.MapSpikeSource("{0}", {1}, {2})\n'\
            .format(device.name, _generate_neurons(device.neurons), info[0])


def extract_structure(transfer_function):
    """
    Converts the given transfer function into a structured format

    :param transfer_function: The transfer function
    :return: A structured format that can be sent via ROS
    """
    try:
        return TransferFunctionASTParser().parse(transfer_function)
    # pylint: disable=broad-except
    except Exception as e:
        logger.exception(e)
        return None


def __get_initial_value_string(var):
    """
    Gets the value string for the initial value for the given variable

    :param var: The variable
    """
    return ('"{}"' if var.type == "str" else '{}').format(var.initial_value)


def generate_code_from_structured_tf(transfer_function):
    """
    Generates PyTF-code from the given transfer function in a structured form

    :param transfer_function:
    :return: The code that represents the transfer function
    """
    imports = []
    devices = ""
    code = "\n"
    return_topic = None
    monitor_device = None
    for dev in transfer_function.devices:
        if transfer_function.type == StructureType.NeuronMonitor and dev.name == "device":
            monitor_device = dev
        else:
            devices += ", " + dev.name
            code += __generate_device(dev)
    for topic in transfer_function.topics:
        if topic.name == "__return__":
            return_topic = topic
        elif transfer_function.type == StructureType.NeuronMonitor and topic.name == "publisher":
            # This topic is inferred by the NeuronMonitor and therefore ignored
            continue
        else:
            devices += ", " + topic.name
            code += __generate_topic(topic)
        topic_type = topic.type[:topic.type.rindex('/')] + ".msg"
        if topic_type not in imports:
            imports.append(topic_type)
    for var in transfer_function.variables:
        devices += ", " + var.name
        code += __generate_variable(var)

    code += __generate_transfer_function_annotation(transfer_function, monitor_device, return_topic)
    code += "def {0}(t{1}):\n".format(transfer_function.name, devices)
    code += indent(transfer_function.code)
    for imp in imports:
        code = "import " + imp + "\n" + code
    return code


def __generate_variable(var):
    """
    Generates the given variable

    :param var: The variable
    :return: The code generated for the given variable
    """
    if var.type == "csv":
        extracted = json.loads(var.initial_value)
        return '@nrp.MapCSVRecorder("{0}", filename="{1}", headers={2})\n'\
            .format(var.name, extracted['filename'], json.dumps(extracted['headers']))
    else:
        return '@nrp.MapVariable("{0}", initial_value={1})\n' \
            .format(var.name, __get_initial_value_string(var))


def __generate_topic_type(topic):
    """
    Generates the topic type

    :param topic: The given topic
    :return: The name of the Python class realizing the topic
    """
    return topic.type.replace("/", ".msg.")


def __generate_topic(topic):
    """
    Generate the code for the given topic

    :param topic: The given topic
    """
    decorator_str = 'nrp.MapRobotPublisher' if topic.publishing else 'nrp.MapRobotSubscriber'

    return '@{decorator}("{0}", Topic("{1}", {2}))\n'\
        .format(topic.name, topic.topic, __generate_topic_type(topic), decorator=decorator_str)


def __generate_transfer_function_annotation(transfer_function, monitor_device, return_topic):
    """
    Generate the main transfer function annotation

    :param monitor_device: The monitor device (only required for neuron monitors)
    :param return_topic: The return topic (only applicable for Neuron2Robot)
    :param transfer_function:  The transfer function
    """
    if transfer_function.type == StructureType.Neuron2Robot:
        if return_topic is None:
            return "@nrp.Neuron2Robot()\n"
        else:
            return '@nrp.Neuron2Robot(Topic("{0}", {1}))\n' \
                .format(return_topic.topic, __generate_topic_type(return_topic))
    elif transfer_function.type == StructureType.Robot2Neuron:
        return "@nrp.Robot2Neuron()\n"
    else:
        monitor_info = device_type_infos[monitor_device.type]
        return "@nrp.NeuronMonitor({0}, {1})\n" \
            .format(_generate_neurons(monitor_device.neurons), monitor_info[0])
