"""
Runs the REST server
"""

__author__ = 'GeorgHinkel'

# pragma: no cover

import argparse
from hbp_nrp_backend.rest_server import app
import logging
import sys

DEFAULT_PORT = 5000
DEFAULT_HOST = '0.0.0.0'


def run_server(server, args):
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

    # We initialize the logging in the startup of the whole backend.
    # This way we can access the already set up logger in the other modules.
    # Also the following configuration can later be easily stored in an external
    # configuration file (and then set by the user).
    logformat = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
    logger = logging.getLogger(__name__)

    try:
        fileHandler = logging.FileHandler(args.logfile)
        fileHandler.setFormatter(logging.Formatter(logformat))
        logger.setLevel(logging.DEBUG)
        logger.addHandler(fileHandler)
    except (AttributeError, IOError) as _:
        consoleHandler = logging.StreamHandler(sys.stdout)
        consoleHandler.setFormatter(logging.Formatter(logformat))
        logger.setLevel(logging.DEBUG)
        logger.addHandler(consoleHandler)
        logger.warn("Could not write to specified logfile or no logfile specified, " +
                    "logging to stdout now!")

    port = DEFAULT_PORT
    try:
        port = int(args.port)
    except (TypeError, ValueError) as _:
        logger.warn("Could not parse port, will use default port: " + str(DEFAULT_PORT))

    logger.info("Starting the REST backend server now ...")
    server.run(port=port, host=DEFAULT_HOST)
    logger.info("REST backend server terminated.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the logfile for the ExDBackend')
    parser.add_argument('--port', dest='port', help='specify the application server\'s port')
    run_server(app, parser.parse_args())
