"""
Code for testing all classes in hbp_nrp_backend.rest_server.__SimulationControl
"""

__author__ = 'Alessandro Ambrosano'

import unittest
import mock
import rospy
import json
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server.__SimulationControl import CustomEventControl, LightControl,\
   UserAuthentication


class MockServiceProxy(object):
    """
    This class mocks the rospy ServiceProxy, its just returns it service name when called.
    """

    def __init__(self, service_name, service_class):
        self.service_name = service_name
        self.service_class = service_class

    def __call__(self, *args, **kwargs):
        TestScript.called_path = self.service_name

    def wait_for_service(self, timeout=0):  #pylint: disable=W0613, R0201
        """
        Mock for ServiceProxy.wait_for_service, it does nothing.
        """
        return None


class TestScript(unittest.TestCase):
    """
    Class for testing hbp_nrp_backend.rest_server.__SimulationControl.
    """

    def setUp(self):
        rospy.ServiceProxy = MockServiceProxy
        rospy.wait_for_service = mock.Mock(return_value=mock.Mock())

        self.cec = CustomEventControl()
        self.lc = LightControl()

        del simulations[:]
        simulations.append(Simulation(0, 'test', 'default-owner', 'created'))

    # The following methods test the class hbp_nrp_backend.rest_server.__SimulationControl
    # .CustomEventControl

    def test_custom_event_control_wrong_user(self):
        """
        This method crafts a request from an user which is not the owner of the simulation.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "wrong-owner"}
        ddict = {}
        with app.test_request_context(headers=hdr):
            self.assertEqual(self.cec.put(0)[1], 401)

    def test_custom_event_control_no_name(self):
        """
        This methods crafts a request from the owner of the simulation but missing the 'name'
        parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.cec.put(0)[1], 400)

    def test_custom_event_control_wrong_name(self):
        """
        This method crafts a request from the owner of the simulation with an invalid 'name'
        parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "randomstring"}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.cec.put(0)[1], 404)  # Wrong name

    def test_custom_event_control_ros_wait_for_service_failure(self):
        """
        This method performs a good request while ROS is unavailable.
        """

        oldwfs = rospy.wait_for_service
        rospy.wait_for_service = lambda x, y: (_ for _ in ()).throw(rospy.ROSException)
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "RightScreenToRed"}
        with app.test_request_context\
                        (headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.cec.put(0)[1], 400)
        rospy.wait_for_service = oldwfs

    def test_custom_event_control_ros_wait_service_proxy_failure(self):
        """
        This method performs a good request with a ROS failure in the service proxy.
        """

        oldsp = rospy.ServiceProxy
        rospy.ServiceProxy = mock.Mock(return_value=lambda model_name='', link_name='',
            visual_name='', property_name='', property_value='': (_ for _ in ()).throw(rospy.ServiceException))
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "RightScreenToRed"}
        with app.test_request_context\
                        (headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.cec.put(0)[1], 400)
        rospy.ServiceProxy = oldsp

    def test_custom_event_control_good_request(self):
        """
        This method crafts some successful requests.
        """

        for name in ["RightScreenToRed", "RightScreenToBlue", "LeftScreenToRed",
             "LeftScreenToBlue"]:
            hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
            ddict = {"name": name}
            with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
                self.assertEquals(self.cec.put(0)[1], 200)


    # The following methods test the class hbp_nrp_backend.rest_server.__SimulationControl
    # .LightControl

    def test_light_control_wrong_user(self):
        """
        This method crafts a request from an user which is not the owner of the simulation.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "wrong-owner"}
        ddict = {}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 401)

    def test_light_control_no_params(self):
        """
        This method crafts a request from the owner of the simulation missing the other parameters.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 400)

    def test_light_control_only_name(self):
        """
        This method crafts a request from the owner of the simulation with only the 'name'
        parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "notreallyimportant"}
        did_it_fail = True
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            try:
                self.lc.put(0)
                did_it_fail = False
            except Exception:
                pass

            if not did_it_fail:
                raise Exception

    def test_light_control_no_attenuation_constant(self):
        """
        This method crafts a request from the owner of the simulation missing only the
        'attenuation_constant' parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_linear": "0",
            "attenuation_quadratic": "0"
        }
        did_it_fail = True
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            try:
                self.lc.put(0)
                did_it_fail = False
            except Exception:
                pass

            if not did_it_fail:
                raise Exception

    def test_light_control_no_attenuation_linear(self):
        """
        This method crafts a request from the owner of the simulation missing only the
        'attenuation_linear' parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_constant": "0",
            "attenuation_quadratic": "0"
        }
        did_it_fail = True
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            try:
                self.lc.put(0)
                did_it_fail = False
            except Exception:
                pass

            if not did_it_fail:
                raise Exception

    def test_light_control_no_attenuation_quadratic(self):
        """
        This method crafts a request from the owner of the simulation missing only the
        'attenuation_quadratic' parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_constant": "0",
            "attenuation_linear": "0"
        }
        did_it_fail = True
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            try:
                self.lc.put(0)
                did_it_fail = False
            except Exception:
                pass

            if not did_it_fail:
                raise Exception

    def test_light_control_no_diffuse(self):
        """
        This method crafts a request from the owner of the simulation missing only the
        'diffuse' parameter.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "attenuation_constant": "0",
            "attenuation_linear": "0",
            "attenuation_quadratic": "0"
        }
        did_it_fail = True
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            try:
                self.lc.put(0)
                did_it_fail = False
            except Exception:
                pass

            if not did_it_fail:
                raise Exception

    def test_good_request_light_control_ros_wait_for_service_failure(self):
        """
        This method performs a good request while ROS is unavailable.
        """

        oldwfs = rospy.wait_for_service
        rospy.wait_for_service = lambda x, y: (_ for _ in ()).throw(rospy.ROSException)
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_constant": "0",
            "attenuation_linear": "0",
            "attenuation_quadratic": "0"
        }
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 400)
        rospy.wait_for_service = oldwfs

    def test_bad_request_light_control_ros_wait_for_service_failure(self):
        """
        This method performs a bad request while ROS is unavailable.
        """

        oldwfs = rospy.wait_for_service
        rospy.wait_for_service = lambda x, y: (_ for _ in ()).throw(rospy.ROSException)
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "notreallyimportant"}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 400)
        rospy.wait_for_service = oldwfs

    def test_good_request_light_control_ros_service_proxy_failure(self):
        """
        This method performs a good request with a ROS failure in the service proxy
        """

        oldsp = rospy.ServiceProxy
        rospy.ServiceProxy = mock.Mock(return_value=lambda light_name='', diffuse='',
                attenuation_constant='', attenuation_linear='', attenuation_quadratic='':
                (_ for _ in ()).throw(rospy.ServiceException))
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_constant": "0",
            "attenuation_linear": "0",
            "attenuation_quadratic": "0"
        }
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 400)
        rospy.ServiceProxy = oldsp

    def test_bad_request_light_control_ros_service_proxy_failure(self):
        """
        This method performs a bad request with a ROS failure in the service proxy
        """

        oldsp = rospy.ServiceProxy
        rospy.ServiceProxy = mock.Mock(return_value=lambda light_name='', diffuse='',
                attenuation_constant='', attenuation_linear='', attenuation_quadratic='':
                (_ for _ in ()).throw(rospy.ServiceException))
        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {"name": "notreallyimportant"}
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 400)
        rospy.ServiceProxy = oldsp

    def test_light_control_good_request(self):
        """
        This method crafts a successful request.
        """

        hdr = {UserAuthentication.HTTP_HEADER_USER_NAME: "default-owner"}
        ddict = {
            "name": "notreallyimportant",
            "diffuse": {
                "r": "0",
                "g": "0",
                "b": "0",
                "a": "0"
           },
            "attenuation_constant": "0",
            "attenuation_linear": "0",
            "attenuation_quadratic": "0"
        }
        with app.test_request_context(headers=hdr, data=json.dumps(ddict)):
            self.assertEqual(self.lc.put(0)[1], 200)

if __name__ == '__main__':
    unittest.main()
