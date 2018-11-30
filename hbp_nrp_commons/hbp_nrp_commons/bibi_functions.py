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
This module provides helper functions for the Brain and Body Integrator model (BIBI)
"""
from hbp_nrp_commons.generated import bibi_api_gen


def get_all_neurons_as_dict(populations):
    """
    Gets the indexing operator for the given neuron selector

    :param populations: All populations
    :return: A dict with population name and slice()/list/None
    """
    pop = {}
    for neurons in populations:
        if isinstance(neurons, bibi_api_gen.Range):
            pop[neurons.population] = slice(neurons.from_,
                                            neurons.to,
                                            getattr(neurons, 'step', None))
        elif isinstance(neurons, bibi_api_gen.List):
            pop[neurons.population] = neurons.element[:]
        elif isinstance(neurons, bibi_api_gen.Population):
            pop[neurons.population] = None
        else:
            raise Exception("Neuron Print: Don't know how to process neuron selector " +
                            str(neurons))
    return pop


def find_changed_strings(list_a, list_b):
    """
    Returns a list that contains changed population names,
    i.e. strings from list_a are searched for in list_b. Not found strings are returned.

    :param list_a: looked up strings to be found in list_b
    :param list_b: looked up strings are compared against strings in list_a

    :return: a list of strings
    """
    return [elem_a for elem_a in list_a if elem_a not in list_b]


def docstring_parameter(*sub):
    """
    Helper functions to include variables in docstrings using the @docstring_parameter decorator

    :param sub: List of variables to be included in the docstring.
    """
    def dec(obj):
        """
        Reformat docstring with variables.
        """
        obj.__doc__ = obj.__doc__.format(*sub)
        return obj

    return dec
