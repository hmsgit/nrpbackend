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
Configuration file sample for the application.
It is overwritten by puppet on dev and staging servers
(see config.py.erb template managed py puppet).
It is used by Jenkins (TestConfig) who sets the password appropriately.
This file handles in particular databases related to Collab contexts ans storage management.
Use the APP_SETTINGS environment variable to switch between configurations, e.g.,:
export APP_SETTINGS=config.LocalConfig
The default value of APP_SETTINGS is config.DeploymentConfig
(managed by puppet via nrp-services-env.sh from server-script)
"""


class Config(object):
    """
    Base config
    SQLALCHEMY_DATABASE_URI is The database URI that should be used for the connection. Examples:
    mysql://username:password@server/db
    sqlite:////tmp/test.db
    More info can be found here: https://pythonhosted.org/Flask-SQLAlchemy/config.html
    """
    SQLALCHEMY_DATABASE_URI = 'postgresql://bbpdbsrv03.epfl.ch:5432/neurorobotics_collab'

    RESTART_SIMULATION_SERVER_COMMAND = ''


class TestConfig(Config):
    """
    Config for unit testing
    Used by Jenkins (see neurorobotics.defaults.yaml from jjb-neurorobotics)
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab_test:2wmAQw7s'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab_test'
    )

    COLLAB_CLIENT_CONFIG = {
        'collab_server': 'https://services.humanbrainproject.eu/collab/v0',
        'oidc_server': 'https://services.humanbrainproject.eu/oidc/',
        'document_server': 'https://services.humanbrainproject.eu/document/v0/api'
    }


class DeploymentConfig(Config):
    """
    Staging (and dev) database config
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab:WRITE_THE_USER_PASSWORD_HERE'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab')

    COLLAB_CLIENT_CONFIG = {
        'collab_server': 'https://services.humanbrainproject.eu/collab/v0',
        'oidc_server': 'https://services.humanbrainproject.eu/oidc/',
        'document_server': 'https://services.humanbrainproject.eu/document/v0/api'
    }


class LocalConfig(Config):

    """
    Database config that allows running the platform locally with a local postgresql database

    Make sure that postgresql is available on the system and that neurorobotics_collab database
    is existing. User postgres, password postgres.
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://postgres:postgres'
        '@localhost:5432/neurorobotics_collab')

    COLLAB_CLIENT_CONFIG = {
        'collab_server': 'https://services.humanbrainproject.eu/collab/v0',
        'oidc_server': 'https://services.humanbrainproject.eu/oidc/',
        'document_server': 'https://services.humanbrainproject.eu/document/v0/api'
    }


class NoDBConfig(Config):

    """
    Dummy database config that allows running the platform locally without having to connect to a
    postgresql server (no collab support then).

    Make sure that sqlite3 as well as pysqlite libraries are available on the dev system.

    Axel: SQLite is not fully supported by SQLAlchemy (no support for ALTER dans DROP), so this
    cannot be used as a real local database.
    """
    SQLALCHEMY_DATABASE_URI = 'sqlite:///:memory:'

    COLLAB_CLIENT_CONFIG = {
        'collab_server': 'https://services.humanbrainproject.eu/collab/v0',
        'oidc_server': 'https://services.humanbrainproject.eu/oidc/',
        'document_server': 'https://services.humanbrainproject.eu/document/v0/api'
    }
