"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'

from hbp_nrp_backend.exd_config import generate_bibi, initialize_experiment
from hbp_nrp_backend.simulation_control import simulations
import os
import logging

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
    simulation = simulations[sim_id]
    experiment = simulation.experiment_id
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    logger.debug("The NRP_MODELS_DIRECTORY is: %s", models_path)
    if models_path is not None:
        experiment = os.path.join(models_path, experiment)
    else:
        logger.warn("NRP_MODELS_DIRECTORY is empty")
    target = '__generated_experiment.py'

    generate_bibi(experiment, target)
    simulation.cle = initialize_experiment(experiment, target)
    logger.info("simulation initialized")
