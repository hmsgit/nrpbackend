"""
Unit tests for the service that patches transfer function sources
"""

__author__ = 'Bernd Eckstein'

import unittest
import json

from mock import MagicMock
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException,\
    NRPServicesTransferFunctionException
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend.rest_server.tests import RestTest
from collections import defaultdict


class DefaultDotDict(defaultdict):
    """
    Creates a Dictionary that can be accessed via dot-Syntax
    e.g. data['more_data'] -> data.more_data
    Overkill but i like it to create my mock return value :)

    Credits go to Zaar Hai
    http://tech.zarmory.com/2013/08/python-putting-dot-in-dict.html
    """
    def __init__(self, *args, **kwargs):
        super(DefaultDotDict, self).__init__(DefaultDotDict)
        if args or kwargs:
            dict.__init__(self, *args, **kwargs)

    def __getattr__(self, attr):
        if attr.startswith('_'):
            raise AttributeError  # Magic only works for key that do not start with "_"
        val = self.__getitem__(attr)
        if isinstance(val, dict) and not isinstance(val, DefaultDotDict):
            val = DefaultDotDict(val)
        self[attr] = val
        return val

    def __setattr__(self, attr, val):
        return self.__setitem__(attr, val)

    def __delattr__(self, attr):
        return self.__delitem__(attr)

    def __deepcopy__(self, memo):
        return DefaultDotDict([(copy.deepcopy(k, memo), copy.deepcopy(v, memo))
                               for k, v in self.items()])


# Test Data
brain_data = DefaultDotDict(brain_data="Data", brain_type="py", data_type="text")
send_data = {'data': "Data", 'brain_type': "py", 'data_type': "text"}
set_ret_ok = DefaultDotDict(error_message="")
set_ret_error = DefaultDotDict(error_message="Crash boom bang", error_line=10, error_column=5)


class TestSimulationTransferFunction(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment_0', None,
                                      'default-owner', 'local', 'created'))
        simulations.append(Simulation(1, 'experiment_1', None,
                                      'untrusted-owner', 'local', 'created'))

        self.sim = simulations[0]
        self.sim.cle = MagicMock()
        self.sim.cle.get_simulation_brain = MagicMock(return_value=brain_data)
        self.sim.cle.set_simulation_brain = MagicMock(return_value=set_ret_ok)

    def test_simulation_brain_get(self):
        response = self.client.get('/simulation/0/brain')
        self.assertEqual(self.sim.cle.get_simulation_brain.call_count, 1)
        self.assertEqual(response.status_code, 200)

        response = self.client.get('/simulation/4/brain')
        self.assertEqual(response.status_code, 404)

    def test_simulation_brain_put(self):
        response = self.client.put('/simulation/0/brain', data=json.dumps(send_data))
        self.assertEqual(self.sim.cle.set_simulation_brain.call_count, 1)
        self.sim.cle.set_simulation_brain.assert_called_with("py", "Data", "text")
        self.assertEqual(response.status_code, 200)

        self.sim.cle.set_simulation_brain = MagicMock(return_value=set_ret_error)
        response = self.client.put('/simulation/0/brain', data=json.dumps(send_data))
        self.assertEqual(response.status_code, 300)

        response = self.client.put('/simulation/4/brain')
        self.assertEqual(response.status_code, 404)

        response = self.client.put('/simulation/1/brain')
        self.assertEqual(response.status_code, 401, "Operation only allowed by simulation owner")


if __name__ == '__main__':
    unittest.main()
