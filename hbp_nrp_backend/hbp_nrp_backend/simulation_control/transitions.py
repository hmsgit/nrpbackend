"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'

from hbp_nrp_backend.exd_config import generate_bibi, initialize_experiment
from hbp_nrp_backend.simulation_control import simulations
from hbp_nrp_backend.rest_server import NRPServicesGeneralException
import os
import logging
import rospy

logger = logging.getLogger(__name__)


def start_simulation(sim_id):
    """
    Starts the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.start()
    logger.info("simulation started")


def pause_simulation(sim_id):
    """
    Pauses the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.pause()
    logger.info("simulation paused")


def reset_simulation(sim_id):
    """
    Reset the simulation with the given simulation id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    # The following two lines are part of a fix for [NRRPLT-1899]
    # To be removed when the following Gazebo issue is solved:
    # https://bitbucket.org/osrf/gazebo/issue/1573/scene_info-does-not-reflect-older-changes
    simulation.left_screen_color = 'Gazebo/Blue'  # pragma: no cover
    simulation.right_screen_color = 'Gazebo/Blue'  # pragma: no cover
    simulation.cle.reset()
    logger.info("simulation reset")


def stop_simulation(sim_id):
    """
    Stops the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.stop()
    logger.info("simulation stopped")


def initialize_simulation(sim_id):
    """
    Releases the simulation with the given id
    :param sim_id: The simulation id
    """
    # generate script
    try:
        simulation = simulations[sim_id]
        experiment = simulation.experiment_id
        gzserver_host = simulation.gzserver_host
        models_path = os.environ.get('NRP_MODELS_DIRECTORY')
        logger.debug("The NRP_MODELS_DIRECTORY is: %s", models_path)
        if models_path is not None:
            experiment = os.path.join(models_path, experiment)
        else:
            logger.warn("NRP_MODELS_DIRECTORY is empty")
        target = '__generated_experiment_%d.py' % (sim_id, )

        generate_bibi(experiment, target, gzserver_host, sim_id)
        simulation.cle = initialize_experiment(experiment, target, sim_id)
        logger.info("simulation initialized")
    except IOError as e:
        raise NRPServicesGeneralException(
            "Error while accessing simulation models (" + e.message + ")",
            "Models error")
    except rospy.ROSException as e:
        raise NRPServicesGeneralException(
            "Error while communicating with the CLE (" + e.message + ")",
            "CLE error")
