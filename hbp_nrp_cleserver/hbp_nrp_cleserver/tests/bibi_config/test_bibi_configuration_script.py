"""
Test for single functions of the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import *
import hbp_nrp_commons.generated.bibi_api_gen as api
import unittest
import pyxb


class TestHelpers(unittest.TestCase):
    def test_is_not_none(self):
        self.assertTrue(is_not_none("Foo"))
        self.assertFalse(is_not_none(None))

    def test_remove_extension(self):
        self.assertEqual(remove_extension("myFile.txt"), "myFile")
        self.assertEqual(remove_extension("foo.bar.txt"), "foo.bar")
        self.assertEqual(remove_extension("foo/bar.txt"), "foo/bar")

class TestEyeSensorTransmit(unittest.TestCase):

    def setUp(self):
        self.directory = os.path.split(__file__)[0]
        milestone2 = os.path.join(self.directory, 'neuronalRedDetection.xml')
        with open(milestone2) as bibi_xml:
            self.config = bibi_api_gen.CreateFromDocument(bibi_xml.read())

        self.eye_sensor_transmit = self.config.transferFunction[0]
        self.assertEqual("eye_sensor_transmit", self.eye_sensor_transmit.name)

    def test_connector_references(self):
        connectors = get_referenced_connectors(self.eye_sensor_transmit, self.config)
        self.assertEqual(2, len(connectors))

    def test_dynamics_references(self):
        dynamics = get_referenced_dynamics(self.eye_sensor_transmit, self.config)
        self.assertEqual(1, len(dynamics))

    def test_tf_generation(self):
        tf_code = generate_tf(self.eye_sensor_transmit, self.config)
        with open(os.path.join(self.directory, "eye_sensor_transmit.txt")) as goal:
            self.assertMultiLineEqual(goal.read(), tf_code)

if __name__ == '__main__':
    unittest.main()
