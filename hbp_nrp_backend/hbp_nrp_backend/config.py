"""
Configuration file for databases
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


class ConfigTest(Config):
    """
    Config for unit testing
    """
    SQLALCHEMY_DATABASE_URI = 'postgresql://fake_db'


class ConfigStaging(Config):
    """
    Staging (and dev) database config
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab:WRITE_THE_USER_PASSWORD_HERE'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab')


class ConfigLocal(Config):
    """
    Database config that allows running the platform locally without having to connect to the
    EPFL postgresql server

    If a persistant storage of the database is desired the following should be changed to a file
    path.

    Make sure that sqlite3 as well as pysqlite libraries are available on the dev system.
    """
    SQLALCHEMY_DATABASE_URI = 'sqlite:///:memory:'
