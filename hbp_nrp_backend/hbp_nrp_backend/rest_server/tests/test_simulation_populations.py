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
Unit tests for the simulation neurons rest call
"""

__author__ = 'Bernd Eckstein'

import unittest
import mock
import rospy
import json
from hbp_nrp_backend.rest_server.tests import RestTest
from collections import defaultdict

ros_service_object = mock.Mock()
rospy.wait_for_service = mock.Mock(return_value=ros_service_object)
rospy.ServiceProxy = mock.Mock(return_value=ros_service_object)

from hbp_nrp_backend.simulation_control import simulations, Simulation

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

neurons = {'populations': [
              {
               'name': "Test",
               'neuron_model': "NeuronModel",
               'parameters': [
                  {'parameterName': "string",
                   'value': 0.0}],
               'gids': [0]
              }
            ]
           }
send_pops_data = {
    'brain_type': 'py',
    'brain_populations': 'populations',
    'data_type': 'text',
    'change_population': 'change'
}
set_ret_ok = DefaultDotDict(message="")
set_ret_error = DefaultDotDict(message="Crash boom bang")

class TestSimulationService(RestTest):

    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'default-owner', 'created'))
        simulations[0].cle = mock.MagicMock()
        simulations[0].cle.set_simulation_populations = mock.MagicMock(return_value=set_ret_ok)
        simulations[0].cle.get_populations = mock.MagicMock(return_value=neurons)

        self.path_can_view = mock.patch('hbp_nrp_backend.__UserAuthentication.UserAuthentication.can_view')
        self.path_can_view.start().return_value = True

    def tearDown(self):
        self.path_can_view.stop()

    def test_get_neurons_sim_ok(self):
        response = self.client.get('/simulation/0/populations')
        self.assertEqual(200, response.status_code)
        # deserialilze
        neurons = json.loads(response.data)
        self.assertEqual("NeuronModel", neurons['populations'][0]['neuron_model'])

    def test_get_neurons_sim_not_found(self):
        response = self.client.get('/simulation/1/populations')
        self.assertEqual(404, response.status_code)

    def test_simulation_brain_put(self):
        response = self.client.put('/simulation/0/populations', data=json.dumps(send_pops_data))
        self.assertEqual(simulations[0].cle.set_simulation_populations.call_count, 1)

        simulations[0].cle.set_simulation_populations.assert_called_with(
            brain_type='py',
            brain_populations= '"populations"',
            data_type='text',
            change_population='change')

        self.assertEqual(response.status_code, 200)

        simulations[0].cle.set_simulation_populations = mock.MagicMock(return_value=set_ret_error)
        response = self.client.put('/simulation/0/populations', data=json.dumps(send_pops_data))
        self.assertEqual(response.status_code, 400)

if __name__ == '__main__':
    unittest.main()
