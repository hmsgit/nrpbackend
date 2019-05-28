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
Test for single functions of the bibi configuration script
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_commons.bibi_functions import *
import hbp_nrp_commons.generated.bibi_api_gen as api
import unittest
import pyxb


class TestScript(unittest.TestCase):

    def test_get_all_neurons_as_dict_range(self):
        populations = [api.Range(population='foo', from_=0, to=2)]
        self.assertEqual(get_all_neurons_as_dict(populations), {'foo':slice(0,2)})
        populations = [api.List(population='foo', element=[1,3])]
        self.assertEqual(get_all_neurons_as_dict(populations), {'foo':[1,3]})
        populations = [api.Population(population='foo', count=5)]
        self.assertEqual(get_all_neurons_as_dict(populations), {'foo':'foo'})
        self.assertRaises(Exception, get_all_neurons_as_dict, ['foo'])

if __name__ == '__main__':
    unittest.main()
