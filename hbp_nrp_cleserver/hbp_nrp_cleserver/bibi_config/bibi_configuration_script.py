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
Script to run CLE from BIBI Configuration File
"""
from hbp_nrp_commons.generated import bibi_api_gen
# We use many of the bibi helper functions here and newly added helper functions should be
# visible to the template(s) immediate, thus the wildcard-import
# pylint: disable=wildcard-import, unused-wildcard-import
from hbp_nrp_commons.bibi_functions import *

__author__ = 'GeorgHinkel'

import os
import re
import logging
import warnings

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

tf_name_regex = re.compile(ur'^.*def\s+(\w+)\s*\(.*', re.MULTILINE)


def deprecated(func): # pragma: no cover
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        """
        Deprecation warning
        """
        message = "Call to deprecated function %s." % func.__name__
        warnings.warn(message, category=DeprecationWarning)
        logger.warn(message)
        return func(*args, **kwargs)

    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

FOUR_SPACES = " " * 4


def correct_indentation(text, first_line_indent_level, indent_string=FOUR_SPACES):
    """
    Adapts the indentation of the given code in order to paste it into the generated script. The
    indentation of the given text is determined based on the first content line (first line which
    consists not only of white spaces). The indentation must either use four spaces or a tab
    character.

    :param text: the original input to adapt
    :param first_line_indent_level: the target indentation level of the entire block
    :type first_line_indent_level: int
    :param indent_string: (Optional): the pattern for one level of indentation
    :return: the adapted text
    """

    text.replace("\t", indent_string)
    lines = text.split("\n")
    result = []

    first_line_indent = None
    for line in lines:
        if line.lstrip():
            if first_line_indent is None and line:
                first_line_stripped = line.lstrip()
                first_line_indent = line[:len(line) - len(first_line_stripped)]

            result.append(indent_string * first_line_indent_level + line[len(first_line_indent):])
        else:
            result.append("\n")

    return "\n".join(result)


def import_referenced_python_tfs(bibi_conf_inst, experiments_path):
    """
    Parses the BIBI configuration, resolves references to external python transfer functions and
    embeds them into the BIBI configuration instance

    :param bibi_conf_inst: A completely parsed 'bibi_api_gen.BIBIconfiguration' instance
    """

    for tf in bibi_conf_inst.transferFunction:
        if hasattr(tf, "src") and tf.src:
            # If the "src" tag is specified, no embedded code is allowed. To make sure
            # clean content beforehand
            assert isinstance(tf, bibi_api_gen.PythonTransferFunction)
            del tf.orderedContent()[:]
            with open(os.path.join(experiments_path, tf.src)) as f:
                tf.append(f.read())


def get_tf_name(tf_code):
    """
    Returns the function name of transfer function code

    @param tf_code: string with transfer function code
    @return: function name
    """

    ret = re.findall(tf_name_regex, tf_code)
    return ret[0] if ret else None


def generate_tf(tf):
    """
    Generates the code for the given transfer function

    :param tf: The transfer function to generate code for
    :return: A unicode string with the generated code
    """
    tf_source = '\n'.join(correct_indentation(cont.value, 1) for cont in tf.orderedContent())
    tf.name = get_tf_name(tf_source)

    return ''.join(l for l in tf_source.splitlines(True) if not l.isspace())
