"""
Runs the REST server
"""
import os

__author__ = 'GeorgHinkel'

import argparse
from hbp_nrp_backend.rest_server import init, app, db
from hbp_nrp_backend.rest_server.cleanup import clean_simulations
from hbp_nrp_backend.rest_server import db_create_and_check, NRPServicesDatabaseTimeoutException
import logging
import sys
import rospy
from threading import Thread

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


# This happens within the app process and so it works both in the command line
# and the uWSGI case
@app.before_first_request
def start_ros():
    """
    Starts ROS utilities and cleanup thread in the app process.
    """
    root_logger.info("Starting ROS node")
    rospy.init_node("nrp_backend")

    rospy_thread = Thread(target=rospy.spin)
    rospy_thread.setDaemon(True)
    rospy_thread.start()

    root_logger.info("Starting cleanup thread")
    cleanup_thread = Thread(target=clean_simulations)
    cleanup_thread.setDaemon(True)
    cleanup_thread.start()


def __process_args():
    """
    Processes the arguments to the server.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the logfile for the ExDBackend')
    parser.add_argument('--port', dest='port', help='specify the application server\'s port')
    parser.add_argument('-p', '--pycharm',
                        dest='pycharm',
                        help='debug with pyCharm. IP adress and port are needed.',
                        nargs='+')
    args = parser.parse_args()

    if args.pycharm:
        # pylint: disable=import-error
        import pydevd

        pydevd.settrace(args.pycharm[0],
                        port=int(args.pycharm[1]),
                        stdoutToServer=True,
                        stderrToServer=True)

    return args


def __init_console_logging():
    """
    Initialize the logging to stdout.
    """

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    root_logger.setLevel(logging.DEBUG)
    root_logger.addHandler(console_handler)

    assert os.environ.get('NRP_MODELS_DIRECTORY') is not None


def init_logging(args):
    """
    Initialize the logging.
    """

    if __name__ == '__main__':
        try:
            file_handler = logging.FileHandler(args.logfile)
            file_handler.setFormatter(logging.Formatter(log_format))
            root_logger.setLevel(logging.DEBUG)
            root_logger.addHandler(file_handler)
            return
        except (AttributeError, IOError) as _:
            __init_console_logging()
            root_logger.warn("Could not write to specified logfile or no logfile specified, " +
                             "logging to stdout now!")
    else:
        # Started with uWSGI or any other framework. Logging is done through the console.
        __init_console_logging()
        root_logger.warn("Application started with uWSGI or any other framework. logging "
                         "to console by default !")


# Populate the data base object
try:
    db_create_and_check(db) # 1 s timeout
except NRPServicesDatabaseTimeoutException as e:
    root_logger.warn("Database connection timeout ( " + str(e) +
                     " ). You are probably in the local mode. ")

init()

if __name__ == '__main__':  # pragma: no cover
    _args = __process_args()
    init_logging(_args)
    port = DEFAULT_PORT
    try:
        port = int(_args.port)
    except (TypeError, ValueError) as _:
        root_logger.warn("Could not parse port, will use default port: " + str(DEFAULT_PORT))
    root_logger.info("Starting the REST backend server now ...")
    app.run(port=port, host=DEFAULT_HOST)
    root_logger.info("REST backend server terminated.")
    rospy.signal_shutdown("Backend terminates")
