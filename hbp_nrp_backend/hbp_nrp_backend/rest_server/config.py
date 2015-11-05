"""
Configuration file for REST services
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
        'postgresql://neurorobotics_collab:NOT_THE_REAL_PASSWORD'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab')
