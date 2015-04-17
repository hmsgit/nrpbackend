"""
This module contains the script to run the integration test
"""

__author__ = 'GeorgHinkel'

import logging
import sys

import rospy
import std_msgs.msg
from cle_ros_msgs.msg import SpikeRate
from hbp_nrp_cle.cle.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClient
from hbp_nrp_cle.bibi_config.bibi_configuration_script import generate_cle
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


def unhandled_exception(type, value, tracebck):
    global failed
    failed = True
    logger.error("Unhandled exception of type {0}: {1}".format(type, value))
    traceback.print_tb(tracebck)

sys.excepthook = unhandled_exception


def monitor_callback(spike_rate):
    global monitor_called
    monitor_called = True
    assert isinstance(spike_rate, SpikeRate)
    if spike_rate.simulationTime < 20:
        if spike_rate.monitorName == "right_wheel_neuron_monitor":
            if not 5 < spike_rate.rate < 30:
                log_message = "Spike rate should have been between 5 and 30" \
                              " but was {0} at simulation time {1}"
                log_message = log_message.format(spike_rate.rate, spike_rate.simulationTime)
                logger.error(log_message)
                global failed
                failed = True

def exception_callback(exc):
    global failed
    failed = True
    logger.exception(exc)
    traceback.print_last()


def check_response(response, msg = "Request", status_code = 200):
    global failed
    if response.status_code != status_code:
        logger.error(msg + " failed, " + repr(response))
        exit(1)
    else:
        logger.info(msg + " successfully completed")


def run_integration_test():
    sim_factory = None
    global failed

    current_dir = os.path.split(__file__)[0]
    generated_bibi_path = os.path.join(current_dir, '__generated_bibi.py')
    run_masked_simulation = os.path.join(current_dir, 'integration_test_simulation_factory.py')

    client = app.test_client()

    try:
        # First, start simulation factory
        sim_factory = subprocess.Popen([sys.executable, run_masked_simulation])

        time.sleep(5)

        os.environ['NRP_MODELS_DIRECTORY'] = current_dir

        rospy.init_node('integration_test_monitor')

        logging.root.addHandler(console_handler)

        rospy.Subscriber("/monitor/population_rate", SpikeRate, monitor_callback)
        rospy.Subscriber("/integration_test/exceptions", std_msgs.msg.String, exception_callback)
        monitor_thread = Thread(target=rospy.spin)

        monitor_thread.start()

        auth_header = {'X-User-Name': "integration-test"}

        # Create experiment
        response = client.post('/simulation', data='{"experimentID":"integration_test_exd.xml"}',
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

        time.sleep(20)

        # Pause the experiment
        response = client.put('/simulation/0/state', data='{"state": "paused"}',
                              headers=auth_header)
        check_response(response, "Pause experiment")

        # Reset the experiment
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

        if os.path.isfile(generated_bibi_path):
            os.remove(generated_bibi_path)


    if failed or not monitor_called:
        if failed:
            logger.error("An error occurred. Marking the integration test to fail.")
        if not monitor_called:
            logger.error("The monitoring callback has not been called. "
                         "Marking integration test as failed.")
        exit(1)
    else:
        logger.info("Integration test succeeded")
        exit(0)

if __name__ == "__main__":
    run_integration_test()