"""
This module contains unit tests for structured transfer functions
"""

import unittest
from cle_ros_msgs.msg import Device, TransferFunction as TF
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

    def test_convert_transfer_function(self):

        test_tf = self.create_default_TF()

        test = StructuredTransferFunction.extract_structure(test_tf)

        self.assert_default_TF(test)

    def test_convert_initialized_transfer_function(self):

        test_tf = self.create_default_TF()

        set_nest_adapter(MockBrainCommunicationAdapter())
        set_robot_adapter(MockRobotCommunicationAdapter())
        initialize("test")

        test = StructuredTransferFunction.extract_structure(test_tf)

        self.assert_default_TF(test)

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
