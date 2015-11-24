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


class TestConfig(Config):
    """
    Config for unit testing
    Used by Jenkins (see neurorobotics.defaults.yaml from jjb-neurorobotics)
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab_test:WRITE_THE_TEST_USER_PASSWORD_HERE'
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
    Database config that allows running the platform locally without having to connect to the
    EPFL postgresql server

    If a persistant storage of the database is desired the following should be changed to a file
    path.

    Make sure that sqlite3 as well as pysqlite libraries are available on the dev system.
    """
    SQLALCHEMY_DATABASE_URI = 'sqlite:///:memory:'

    COLLAB_CLIENT_CONFIG = {
        'collab_server': 'https://services.humanbrainproject.eu/collab/v0',
        'oidc_server': 'https://services.humanbrainproject.eu/oidc/',
        'document_server': 'https://services.humanbrainproject.eu/document/v0/api'
    }
