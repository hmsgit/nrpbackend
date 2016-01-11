"""
Configuration file for testing the database connection
"""

__author__ = "Luc Guyot"

class LocalConfigTest():

    """
    Database config that allows running the platform locally without having to connect to the
    EPFL postgresql server

    If a persistant storage of the database is desired the following should be changed to a file
    path.

    Make sure that sqlite3 as well as pysqlite libraries are available on the dev system.
    """
    SQLALCHEMY_DATABASE_URI = 'sqlite:///:memory:'
