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
This module inspects a transfer function and returns a formal object describing it
"""

__author__ = 'Claudio Sousa'

from cle_ros_msgs.msg import TransferFunction, Device, Topic, Variable, ExperimentPopulationInfo
import ast
import json
import textwrap
import logging

logger = logging.getLogger(__name__)


class StructureType(object):

    """
    Transfer function type
    """
    Neuron2Robot = 2
    Robot2Neuron = 1
    NeuronMonitor = 3


device_types = {
    'ac_source': 'ACSource',
    'dc_source': 'DCSource',
    'fixed_frequency': 'FixedFrequency',
    'leaky_integrator_alpha': 'LeakyIntegratorAlpha',
    'leaky_integrator_exp': 'LeakyIntegratorExp',
    'nc_source': 'NCSource',
    'poisson': 'Poisson',
    'spike_recorder': 'SpikeRecorder',
    'population_rate': 'PopulationRate',
    'raw_signal': 'RawSignal'
}


class TransferFunctionASTParser(object):

    """
    Transfer function source code parser that uses the generated AST to instrospect the TF structure
    """

    def __init__(self):
        self._tf = TransferFunction()
        self._imports = dict()

    def parse(self, code):
        """
        Parses a TF code source

        :returns: the structured TF
        """
        tfAst = ast.parse(code)

        self._tf.code = TransferFunctionASTParser.__get_tf_code(code)

        self._visitAST(tfAst)
        return self._tf

    def _visitAST(self, tfAst):
        """
        Visits the TF generated AST
        """
        functions = [instr for instr in tfAst.body if isinstance(instr, ast.FunctionDef)]
        importsFrom = [instr for instr in tfAst.body if isinstance(instr, ast.ImportFrom)]

        for importFrom in importsFrom:
            self._visit_ImportFrom(importFrom)

        assert len(functions) == 1, 'Expected one function on TF, found %i' % len(functions)

        self._visit_FunctionDef(functions[0])

    def _visit_ImportFrom(self, importFrom):
        """
        Handles import AST nodes (ie 'from ... import [as ...]')
        """
        module = importFrom.module
        for nameNode in importFrom.names:
            alias = nameNode.asname if nameNode.asname else nameNode.name
            self._imports[alias] = (nameNode.name, module)

    def _visit_FunctionDef(self, functionNode):
        """
        Handles function definition AST node
        """
        self._tf.name = functionNode.name

        for decorator in functionNode.decorator_list:
            self._visit_decorator(decorator)

    def _visit_decorator(self, decorator):
        """
        Handles a function decorator
        """
        decorator_name = self._get_attribute_path(decorator.func)[-1]
        method = '_visit_' + decorator_name
        visitor = getattr(self, method, self._visit_UndeclaredDecorator)
        visitor(decorator)

    def _visit_MapRobotPublisher(self, decorator):
        """
        Handles MapRobotPublisher decorators
        """
        self._visit_publisher(decorator)

    def _visit_publisher(self, decorator):
        """
        Handles generic publisher decorators
        """
        assert len(
            decorator.args) == 2, '%s: expected two arguments, found %i' % (decorator.func.attr,
                                                                            len(decorator.args))

        topic = self._visit_Topic(decorator.args[1])
        topic.name = TransferFunctionASTParser._get_value(decorator.args[0])
        topic.publishing = True

        self._tf.topics.append(topic)

    def _visit_subscriber(self, decorator):
        """
        Handles generic subscriber decorators
        """
        assert len(
            decorator.args) == 2, '%s: expected two arguments, found %i' % (decorator.func.attr,
                                                                            len(decorator.args))

        topic = self._visit_Topic(decorator.args[1])
        topic.name = TransferFunctionASTParser._get_value(decorator.args[0])
        topic.publishing = False

        self._tf.topics.append(topic)

    def _visit_MapSpikeSource(self, decorator):
        """
        Handles MapSpikeSource decorators
        """
        self._visit_DeviceDecorator(decorator)

    def _visit_MapSpikeSink(self, decorator):
        """
        Handles MapSpikeSink decorators
        """
        self._visit_DeviceDecorator(decorator)

    def _visit_DeviceDecorator(self, decorator):
        """
        Handles Device decorators
        """
        assert len(
            decorator.args) == 3, "%s: expected 3 args, found %i" % (self.str_node(decorator),
                                                                     len(decorator.args))

        device_name = self._get_attribute_path(decorator.args[2])[-1]

        device = Device()
        device.type = device_types[device_name]
        device.name = TransferFunctionASTParser._get_value(decorator.args[0])
        device.neurons = self._get_neurons(decorator.args[1])
        self._tf.devices.append(device)

    def _visit_NeuronMonitor(self, decorator):
        """
        Handles NeuronMonitor decorators
        """
        self._tf.type = StructureType.NeuronMonitor

        device_name = self._get_attribute_path(decorator.args[1])[-1]

        topic = Topic()
        topic.topic = '/monitor/' + device_name
        topic.type = 'cle_ros_msgs/' + \
            ('SpikeEvent' if device_name == 'spike_recorder' else 'SpikeRate')
        topic.name = 'publisher'
        topic.publishing = True
        self._tf.topics.append(topic)

        device = Device()
        device.type = device_types[device_name]
        device.name = 'device'
        device.neurons = self._get_neurons(decorator.args[0])

        self._tf.devices.append(device)

    def _get_neurons(self, neuron_node):
        """
        Parses the Population information

        :returns: the list of neurons and population name
        """
        neurons = ExperimentPopulationInfo()

        if type(neuron_node) == ast.Subscript:
            pop_path = self._get_attribute_path(neuron_node.value)
            neurons.name = pop_path[-1]
            neurons.start, neurons.stop, neurons.step = self._get_slice(neuron_node.slice)
            neurons.type = 1
        elif type(neuron_node) == ast.Attribute:
            pop_path = self._get_attribute_path(neuron_node)
            neurons.name = pop_path[-1]
            neurons.start, neurons.stop, neurons.step = [0, 0, 0]
            neurons.type = 0
        else:
            neurons.name = "Custom logic"
            neurons.start, neurons.stop, neurons.step = [0, 0, 0]
            neurons.type = 0

        return neurons

    def _get_slice(self, index):
        """
        Parses a slice instance (eg. slice(1, 4, 1)), an index (eg. [1]) or a slice (eg. [1:4:1])
        """
        if type(index) == ast.Index:
            if (type(index.value) == ast.Call and type(index.value.func) == ast.Name and
                    index.value.func.id == "slice"):
                # eg: slice(1, 4, 1)
                args = [TransferFunctionASTParser._get_value(arg) for arg in index.value.args]
                if len(args) == 2:
                    args.append(1)
                return args
            elif type(index.value) == ast.Num:
                # eg: [1]
                num = TransferFunctionASTParser._get_value(index.value)
                return [num, num + 1, 1]
            else:
                raise Exception("_get_slice unknown index value %s" % self.str_node(index.value))
        elif type(index) == ast.Slice:
            # [1:4:1]
            return map(self._get_value, [index.lower, index.upper, index.step])
        else:
            raise Exception("get_slice unexpected argument, got %s" % self.str_node(index))

    # pylint: disable=unused-argument
    def _visit_Robot2Neuron(self, decorator):
        """
        Handles the Robot2Neuron decorator
        """
        self._tf.type = StructureType.Robot2Neuron

    def _visit_Neuron2Robot(self, decorator):
        """
        Handles the Neuron2Robot decorator
        """
        if decorator.args:
            assert len(decorator.args) == 1, 'Neuron2Robot: expected one arguments, found %i' % len(
                decorator.args)
            topic = self._visit_Topic(decorator.args[0])
            topic.name = '__return__'
            topic.publishing = True
            self._tf.topics.append(topic)

        self._tf.type = StructureType.Neuron2Robot

    def _visit_MapRobotSubscriber(self, decorator):
        """
        Handles the MapRobotSubscriber decorator
        """
        self._visit_subscriber(decorator)

    def _visit_Topic(self, node):
        """
        Parses the Topic AST node and returns it's information
        """
        assert type(node) == ast.Call and node.func.id == 'Topic', \
            'expected a Topic instantiation, found %s' % \
            self.str_node(node)
        assert len(node.args) == 2, 'Expected Topic to have two arguments'

        topic = Topic()
        # print(self.str_node(node.args[1]))
        topic.topic = TransferFunctionASTParser._get_value(node.args[0])
        topic.type = self._get_topic_ros_type(node.args[1])

        return topic

    def _visit_MapCSVRecorder(self, decorator):
        """
        Handles the MapCSVRecorder decorator
        """
        variable = Variable()
        variable.name = TransferFunctionASTParser._get_value(decorator.args[0])
        variable.type = "csv"
        initialValue = {}
        if len(decorator.args) >= 2:
            initialValue['filename'] = TransferFunctionASTParser._get_value(decorator.args[1])
        if len(decorator.args) >= 3:
            initialValue['headers'] = TransferFunctionASTParser._get_list_value(decorator.args[2])
        for keyword in decorator.keywords:
            if keyword.arg == 'filename':
                initialValue['filename'] = TransferFunctionASTParser._get_value(keyword.value)
            elif keyword.arg == 'headers':
                initialValue['headers'] = TransferFunctionASTParser._get_list_value(keyword.value)
        variable.initial_value = json.dumps(initialValue)

        self._tf.variables.append(variable)

    def _visit_MapVariable(self, decorator):
        """
        Handles the MapVariable decorator
        """
        assert len(decorator.args) == 1, \
            'Expected MapVariable %s to have at one argument, found %i' \
            % (decorator.func.attr, len(decorator.args))
        variable = Variable()
        variable.name = TransferFunctionASTParser._get_value(decorator.args[0])
        for keyword in decorator.keywords:
            if keyword.arg == 'initial_value':
                initial_value = TransferFunctionASTParser._get_value(keyword.value)
                variable.initial_value = str(initial_value)
                variable.type = type(initial_value).__name__

        self._tf.variables.append(variable)

    def _get_topic_ros_type(self, node):
        """
        Parses the ROS topic type

        :returns: the ROS topic type
        """
        path = self._get_attribute_path(node)
        if path[0] in self._imports:
            name, module = self._imports[path[0]]
            path = module.split('.') + [name] + path[1:]
        if len(path) == 3:
            path = path[::2]
        return '/'.join(path)

    def _get_attribute_path(self, node):
        """
        Parses an attribute AST node

        :returns: list of string words in the attributs
        """
        if type(node) == ast.Name:
            return [TransferFunctionASTParser._get_value(node)]

        path = self._get_attribute_path(node.value)
        path.append(node.attr)
        return path

    @staticmethod
    def __get_tf_code(tf):
        '''
        Extracts the body from the given in-memory transfer function

        :param tf: The transfer function
        :return: The extracted body of the transfer function
        '''
        import re
        fnbody = re.search(r'.*\ndef +[a-zA-Z0-9]* *[^\)]*[^\n]*(?P<body>(\n( [^\n]*|))*)', tf)
        if fnbody:
            fnbody = fnbody.group('body')
        if not fnbody:
            return tf.source
        return textwrap.dedent(fnbody[1:]).rstrip('\n')

    @staticmethod
    def _get_list_value(node):
        """
        Parses a list AST node
        """
        return [TransferFunctionASTParser._get_value(v) for v in node.elts]

    @staticmethod
    def _get_value(node):
        """
        Parses a Value/Name AST Node
        """
        scalarNames = {'None': None, 'False': False, 'True': True}
        scalarObjects = {
            'Num': lambda o: o.n,
            'Str': lambda o: o.s,
            'Name': lambda o: scalarNames[o.id] if o.id in scalarNames else o.id
        }
        class_name = node.__class__.__name__

        assert class_name in scalarObjects, 'Unknown scalar class %s' % class_name

        return scalarObjects[class_name](node)

    def _visit_UndeclaredDecorator(self, decorator):
        """
        Fallback when a unknown decorator is found
        """
        logger.debug('Undeclared decorator: ' + decorator.func.attr)
        logger.debug(self.str_node(decorator.func))
        logger.debug('Args::')
        for arg in decorator.args:
            logger.debug('\t-: ' + self.str_node(arg))
        logger.debug('keywords::')
        for keyword in decorator.keywords:
            logger.debug('\t-: ' + self.str_node(keyword))

    def str_node(self, node):
        """
        Stringifies an AST node
        """
        if isinstance(node, ast.AST):
            fields = [(name, self.str_node(val)) for name, val in ast.iter_fields(node)
                      if name not in ('left', 'right')]
            rv = '%s(%s' % (node.__class__.__name__, ', '.join('%s=%s' % field for field in fields))
            return rv + ')'
        else:
            return repr(node)
