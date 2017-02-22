"""
This module contains unit tests for structured transfer functions
"""

import unittest
from mock import patch, MagicMock
from cle_ros_msgs.msg import Device, ExperimentPopulationInfo, TransferFunction as TF
from hbp_nrp_cle.tf_framework import *
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cleserver.bibi_config import StructuredTransferFunction

__author__ = "Georg Hinkel"

class TestStructuredTransferFunctions(unittest.TestCase):

    def setUp(self):
        start_new_tf_manager()

    def create_default_TF(self):
        @MapRobotPublisher("pub", Topic("/foo", Device))
        @MapSpikeSink("neuron", brain.sensors[4], leaky_integrator_alpha)
        @Neuron2Robot(Topic('/bar', TF))
        def test_tf(t, pub, neuron):
            print '42'

        return test_tf

    def create_other_TF(self):
        @MapRobotSubscriber("sub", Topic("/foo", Device))
        @MapSpikeSource("neuron", brain.actors[1:4:2], poisson)
        @Robot2Neuron()
        def test_tf_2(t, sub, neuron):
            print '42'

        return test_tf_2

    def assert_default_TF(self, test):
        self.assertIsNotNone(test)
        self.assertEqual("test_tf", test.name)
        self.assertEqual("print '42'", test.code)
        self.assertEqual(TF.NEURON2ROBOT, test.type)
        self.assertEqual(1, len(test.devices))
        self.assertEqual(2, len(test.topics))
        self.assertEqual(0, len(test.variables))
        d = test.devices[0]
        t = test.topics[0]
        self.assertEqual("neuron", d.name)
        self.assertEqual("LeakyIntegratorAlpha", d.type)
        self.assertEqual("pub", t.name)
        self.assertEqual("/foo", t.topic)
        self.assertEqual("cle_ros_msgs.msg.Device", t.type)
        self.assertTrue(t.publishing)

    def assert_other_TF(self, test):
        self.assertIsNotNone(test)
        self.assertEqual("test_tf_2", test.name)
        self.assertEqual("print '42'", test.code)
        self.assertEqual(TF.ROBOT2NEURON, test.type)
        self.assertEqual(1, len(test.devices))
        self.assertEqual(1, len(test.topics))
        self.assertEqual(0, len(test.variables))
        d = test.devices[0]
        t = test.topics[0]
        self.assertEqual("neuron", d.name)
        self.assertEqual("Poisson", d.type)
        self.assertEqual("sub", t.name)
        self.assertEqual("/foo", t.topic)
        self.assertEqual("cle_ros_msgs.msg.Device", t.type)
        self.assertFalse(t.publishing)

    def test_convert_transfer_function(self):

        test_tf = self.create_default_TF()

        test = StructuredTransferFunction.extract_structure(test_tf)

        self.assert_default_TF(test)

        test_tf_2 = self.create_other_TF()
        test_2 = StructuredTransferFunction.extract_structure(test_tf_2)
        self.assert_other_TF(test_2)

    def test_convert_initialized_transfer_function(self):

        test_tf = self.create_default_TF()
        test_tf_2 = self.create_other_TF()

        set_nest_adapter(MockBrainCommunicationAdapter())
        set_robot_adapter(MockRobotCommunicationAdapter())
        initialize("test")

        test = StructuredTransferFunction.extract_structure(test_tf)
        test2 = StructuredTransferFunction.extract_structure(test_tf_2)

        self.assert_default_TF(test)
        self.assert_other_TF(test2)

    @patch('hbp_nrp_cleserver.bibi_config.StructuredTransferFunction.ExperimentPopulationInfo')
    def test_extract_neurons(self, mock_exp_info):
        mock_exp_info.TYPE_ENTIRE_POPULATION = "type"
        mock_exp_info.TYPE_POPULATION_SLICE = "slice"
        mock_exp_info.TYPE_POPULATION_LISTVIEW = "list"
        neurons = MagicMock(spec=['name'])
        neurons.name = "neuron_name"
        StructuredTransferFunction._extract_neurons(neurons)

        mock_exp_info.assert_called_with(name=neurons.name, type=mock_exp_info.TYPE_ENTIRE_POPULATION,
                                         ids=[], start=0, stop=0, step=0)
        neurons = MagicMock(spec=['name', 'index', 'parent'])
        neurons.index = 5
        neurons.parent = neurons
        StructuredTransferFunction._extract_neurons(neurons)
        mock_exp_info.assert_called_with(name=neurons.name, type=mock_exp_info.TYPE_POPULATION_SLICE,
                                         ids=[], start=neurons.index, stop=neurons.index+1, step=1)
        neurons.index = [5]
        StructuredTransferFunction._extract_neurons(neurons)
        mock_exp_info.assert_called_with(name=neurons.name, type=mock_exp_info.TYPE_POPULATION_LISTVIEW,
                                         ids=neurons.index, start=0, stop=0, step=0)
        with self.assertRaises(Exception):
            neurons.index = {}
            StructuredTransferFunction._extract_neurons(neurons)

    def test_generate_neurons(self):
        neurons = MagicMock()
        neurons.name = "neuron_name"
        neurons.start = 0
        neurons.type = ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION

        self.assertEqual(StructuredTransferFunction._generate_neurons(neurons), "nrp.brain.neuron_name")

        neurons.type = ExperimentPopulationInfo.TYPE_POPULATION_SLICE
        neurons.stop = neurons.start + 1
        self.assertEqual(StructuredTransferFunction._generate_neurons(neurons), "nrp.brain.neuron_name[0]")

        neurons.stop = 100
        neurons.step = 5
        self.assertEqual(StructuredTransferFunction._generate_neurons(neurons), "nrp.brain.neuron_name[slice(0,100,5)]")
        neurons.type = ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW
        neurons.ids = ['1','2','3']
        self.assertEqual(StructuredTransferFunction._generate_neurons(neurons), "nrp.brain.neuron_name[[1, 2, 3]]")

    def test_roundtrip(self):

        test_tf = self.create_default_TF()
        converted = StructuredTransferFunction.extract_structure(test_tf)
        code = StructuredTransferFunction.generate_code_from_structured_tf(converted)

        self.assertMultiLineEqual(
            '@nrp.MapSpikeSink("neuron", nrp.brain.sensors[4], nrp.leaky_integrator_alpha)\n'
            '@nrp.MapRobotPublisher("pub", Topic("/foo", cle_ros_msgs.msg.Device))\n'
            '@nrp.Neuron2Robot(Topic("/bar", cle_ros_msgs.msg.TransferFunction))\n'
            'def test_tf(t, neuron, pub):\n'
            '    print \'42\'', code)

        test_tf = self.create_other_TF()
        converted = StructuredTransferFunction.extract_structure(test_tf)
        code = StructuredTransferFunction.generate_code_from_structured_tf(converted)

        self.assertMultiLineEqual(
            '@nrp.MapSpikeSource("neuron", nrp.brain.actors[slice(1,4,2)], nrp.poisson)\n'
            '@nrp.MapRobotSubscriber("sub", Topic("/foo", cle_ros_msgs.msg.Device))\n'
            '@nrp.Robot2Neuron()\n'
            'def test_tf_2(t, neuron, sub):\n'
            '    print \'42\'', code)
