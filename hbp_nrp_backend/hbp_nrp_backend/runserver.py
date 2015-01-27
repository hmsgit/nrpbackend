"""
Runs the REST server
"""

__author__ = 'GeorgHinkel'

# pragma: no cover

import argparse
from hbp_nrp_backend.rest_server import app
import logging
import sys

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the logfile for the ExDBackend')
    parser.add_argument('--port', dest='port', help='specify the application server\'s port')
    args = parser.parse_args()

    # We initialize the logging in the startup of the whole backend.
    # This way we can access the already set up logger in the other modules.
    # Also the following configuration can later be easily stored in an external
    # configuration file (and then set by the user).
    logformat = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
    logger = logging.getLogger(__name__)

    try:
        logging.basicConfig(filename=args.logfile,
                            filemode='w',
                            format=logformat,
                            level=logging.DEBUG)
    except IOError:
        consoleHandler = logging.StreamHandler(sys.stdout)
        consoleHandler.setFormatter(logging.Formatter(logformat))
        logger.setLevel(logging.DEBUG)
        logger.addHandler(consoleHandler)
        logger.warn("Could not write to specified logfile, logging to stdout now!")

    port = 5000
    try:
        port = int(args.port)
    except (TypeError, ValueError) as exc:
        logger.info("Could not parse port, using default port 5000")

    logger.info("Starting the REST backend server now ...")
    app.run(port=port, host='0.0.0.0')
    logger.info("REST backend server terminated.")
