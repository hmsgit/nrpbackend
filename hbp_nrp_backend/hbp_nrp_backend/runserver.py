"""
Runs the REST server
"""
import os

__author__ = 'GeorgHinkel'

import argparse
from hbp_nrp_backend.rest_server import app
import logging
import sys

DEFAULT_PORT = 5000
DEFAULT_HOST = '0.0.0.0'

# We initialize the logging in the startup of the whole backend.
# This way we can access the already set up logger in the other modules.
# Also the following configuration can later be easily stored in an external
# configuration file (and then set by the user).
log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
# Warning: We do not use __name__  here, since it translates to "__main__"
# when this file is ran directly (such as python runserver.py)
root_logger = logging.getLogger('hbp_nrp_backend')


def run_server(server, args):  # pragma: no cover
    """
    Runs the given server with the given arguments. Within the args parameter one
    can specify 'logfile' as well as the 'port' where the server runs. In case
    there is no valid logfile present the server logs to stdout. In case no valid
    port is given a default port, defined as a constant above, is used.
    (This function takes those two arguments in order to be able to inject mocks
    easily and hence make the function easy to test.)
    :param server: The server on which the run method is called
    :param args: The parsed arguments
    """

    try:
        file_handler = logging.FileHandler(args.logfile)
        file_handler.setFormatter(logging.Formatter(log_format))
        root_logger.setLevel(logging.DEBUG)
        root_logger.addHandler(file_handler)
    except (AttributeError, IOError) as _:
        __init_console_logging()
        root_logger.warn("Could not write to specified logfile or no logfile specified, " +
                         "logging to stdout now!")

    port = DEFAULT_PORT
    try:
        port = int(args.port)
    except (TypeError, ValueError) as _:
        root_logger.warn("Could not parse port, will use default port: " + str(DEFAULT_PORT))

    root_logger.info("Starting the REST backend server now ...")
    # server.debug=True
    server.run(port=port, host=DEFAULT_HOST)
    root_logger.info("REST backend server terminated.")


def __init_console_logging():
    """
    Initialize the logging to stdout.
    """

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    root_logger.setLevel(logging.DEBUG)
    root_logger.addHandler(console_handler)

    assert os.environ.get('NRP_MODELS_DIRECTORY') is not None

if __name__ == '__main__':  # pragma: no cover
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the logfile for the ExDBackend')
    parser.add_argument('--port', dest='port', help='specify the application server\'s port')
    run_server(app, parser.parse_args())
else:
    # Started with uWSGI or any other framework. Logging is done through the console.
    __init_console_logging()
    root_logger.warn("Application started with uWSGI or any other framework. logging "
                     "to console by default !")
