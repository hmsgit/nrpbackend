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
