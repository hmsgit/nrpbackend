"""
This module contains the script to run the integration test
"""

__author__ = 'GeorgHinkel'

import logging
import sys

import rospy
import std_msgs.msg
from cle_ros_msgs.msg import SpikeRate
from threading import Thread
import json
import os
import time
import traceback
import subprocess
from hbp_nrp_backend.rest_server import app


log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
logger = logging.getLogger(__name__)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(logging.Formatter(log_format))
logging.root.setLevel(logging.DEBUG)
logging.root.addHandler(console_handler)

failed = False
monitor_called = False
has_seen_red = False

error_message = None

min_spike_rate_right = 10000
max_spike_rate_right = 0
min_spike_rate_left = 10000
max_spike_rate_left = 0


def unhandled_exception(type, value, tracebck):
    global failed
    failed = True
    global error_message
    error_message = value
    logger.error("Unhandled exception of type {0}: {1}".format(type, value))
    traceback.print_tb(tracebck)

sys.excepthook = unhandled_exception


def monitor_callback(spike_rate):
    global monitor_called
    global has_seen_red
    global min_spike_rate_right
    global max_spike_rate_right
    global min_spike_rate_left
    global max_spike_rate_left
    monitor_called = True
    assert isinstance(spike_rate, SpikeRate)
    if 4 < spike_rate.simulationTime < 30:
        if spike_rate.monitorName == "right_wheel_neuron_monitor":
            if not has_seen_red and spike_rate.rate < 30:
                log_message = "Spike rate should have been at least 30 as Husky has not found red" \
                              " but was {0} at simulation time {1}"
                log_message = log_message.format(spike_rate.rate, spike_rate.simulationTime)
                logger.error(log_message)
                global failed
                failed = True
                global error_message
                error_message = log_message
        if spike_rate.rate < min_spike_rate_right:
            min_spike_rate_right = spike_rate.rate
        if spike_rate.rate > max_spike_rate_right:
            max_spike_rate_right = spike_rate.rate
    if spike_rate.monitorName == "left_wheel_neuron_monitor":
        if spike_rate.rate > 50 and not has_seen_red:
            logger.info("Population on left action neuron detected.")
            logger.info("Assuming Husky has detected red color.")
            has_seen_red = True
        if spike_rate.rate < min_spike_rate_left:
            min_spike_rate_left = spike_rate.rate
        if spike_rate.rate > max_spike_rate_left:
            max_spike_rate_left = spike_rate.rate


def exception_callback(exc):
    global failed
    failed = True
    global error_message
    error_message = repr(exc)
    logger.exception(exc)
    traceback.print_last()


def check_response(response, msg="Request", status_code=200):
    if response.status_code != status_code:
        logger.error(msg + " failed, " + response.data)
        exit(1)
    else:
        logger.info(msg + " successfully completed")


def run_integration_test():
    sim_factory = None
    global failed

    current_dir = os.path.split(__file__)[0]
    run_masked_simulation = os.path.join(current_dir, 'integration_test_simulation_factory.py')

    client = app.test_client()

    try:
        # First, start simulation factory
        sim_factory = subprocess.Popen([sys.executable, run_masked_simulation])

        time.sleep(5)

        rospy.init_node('integration_test_monitor')

        logging.root.addHandler(console_handler)

        rospy.Subscriber("/monitor/population_rate", SpikeRate, monitor_callback)
        rospy.Subscriber("/integration_test/exceptions", std_msgs.msg.String, exception_callback)
        monitor_thread = Thread(target=rospy.spin)

        monitor_thread.start()

        auth_header = {'X-User-Name': "integration-test"}

        # Create experiment
        rqdata = {
            "experimentConfiguration": "integration_test/integration_test_exd.xml",
            "gzserverHost": "local"
        }
        response = client.post('/simulation',
                               data=json.dumps(rqdata),
                               headers=auth_header)
        check_response(response, "Create experiment", status_code=201)

        # Initialize experiment
        response = client.put('/simulation/0/state', data='{"state": "initialized"}',
                              headers=auth_header)
        check_response(response, "Initialize experiment")

        # Change color of the light
        light_data = {
            "name": "right_spot",
            "diffuse": {
                "r": "255",
                "g": "255",
                "b": "255",
                "a": "255"
            }
        }
        response = client.put('/simulation/0/interaction/light',
                              data=json.dumps(light_data, ensure_ascii=True),
                              headers=auth_header)
        check_response(response, "Set color of the light")

        # Set right screen to red
        response = client.put('/simulation/0/interaction', data='{"name": "RightScreenToRed"}',
                              headers=auth_header)
        check_response(response, "Set monitor to red")

        # Start experiment
        response = client.put('/simulation/0/state', data='{"state": "started"}',
                              headers=auth_header)
        check_response(response, "Start experiment")

        time.sleep(30)

        # Pause the experiment
        response = client.put('/simulation/0/state', data='{"state": "paused"}',
                              headers=auth_header)
        check_response(response, "Pause experiment")

        # Reset the experiment, the new way
        response = client.put('/simulation/0/reset',
                              data=json.dumps({'oldReset': False,
                                               'robotPose': False,
                                               'fullReset': False}),
                              headers=auth_header)
        check_response(response, "Reset Experiment")

        # Start experiment again
        response = client.put('/simulation/0/state', data='{"state": "started"}',
                              headers=auth_header)
        check_response(response, "Start experiment")

         # Pause the experiment again
        response = client.put('/simulation/0/state', data='{"state": "paused"}',
                              headers=auth_header)
        check_response(response, "Pause experiment")

        # Reset the experiment, the old way
        response = client.put('/simulation/0/state', data='{"state": "initialized"}',
                              headers=auth_header)
        check_response(response, "Reset experiment")

        # Stop experiment
        response = client.put('/simulation/0/state', data='{"state": "stopped"}',
                              headers=auth_header)
        check_response(response, "Stop experiment")

    finally:
        if sim_factory is not None:
            sim_factory.kill()

        # This signal shutdown will only shutdown the monitoring subscriber, not a running gzserver
        rospy.signal_shutdown("Integration test is done")

    logger.info("Left actor neuron had spike rates between {0} and {1}"
                .format(min_spike_rate_left, max_spike_rate_left))
    logger.info("Right actor neuron had spike rates between {0} and {1}"
                .format(min_spike_rate_right, max_spike_rate_right))

    if failed or not monitor_called or not has_seen_red:
        logger.error("An error occurred. Marking the integration test to fail.")
        if not has_seen_red:
            logger.error("Husky failed to detect red color")
        if failed:
            logger.error("Last error message was: " + error_message)
        if not monitor_called:
            logger.error("The monitoring callback has not been called. ")
        exit(1)
    else:
        logger.info("Integration test succeeded")
        exit(0)

if __name__ == "__main__":
    run_integration_test()
