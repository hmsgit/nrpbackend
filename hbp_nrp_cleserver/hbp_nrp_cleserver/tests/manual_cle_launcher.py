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
This module is meant as a debugging helper to manually start the cle
"""

from hbp_nrp_commons.generated import exp_conf_api_gen as exd
from hbp_nrp_commons.generated import bibi_api_gen as bibi
from hbp_nrp_cleserver.server import CLELauncher
from os import environ, path

__author__ = 'Georg Hinkel'

def create_cle_launcher(config):
    models_path = environ.get('NRP_MODELS_DIRECTORY')

    with open(path.join(models_path, config)) as exd_file:
        exd_config = exd.CreateFromDocument(exd_file.read())

    with open(path.join(models_path, exd_config.bibiConf)) as bibi_file:
        bibi_config = bibi.CreateFromDocument(bibi_file.read())

    cle_launcher = CLELauncher(exd_config, bibi_config, models_path, 'local', 0)
    return cle_launcher, exd_config, bibi_config