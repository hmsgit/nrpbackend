"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'

from hbp_nrp_backend.exd_config import generate_bibi, initialize_experiment
from hbp_nrp_backend.simulation_control import simulations
import os


def start_simulation(sim_id):
    """
    Starts the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.start()


def pause_simulation(sim_id):
    """
    Pauses the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.pause()


def reset_simulation(sim_id):
    """
    Reset the simulation with the given simulation id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.reset()


def stop_simulation(sim_id):
    """
    Stops the simulation with the given id
    :param sim_id: The simulation id
    """
    simulation = simulations[sim_id]
    simulation.cle.stop()


def initialize_simulation(sim_id):
    """
    Releases the simulation with the given id
    :param sim_id: The simulation id
    """
    # generate script
    simulation = simulations[sim_id]
    experiment = simulation.experiment_id
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    if models_path is not None:
        experiment = os.path.join(models_path, experiment)
    target = '__generated_experiment.py'

    generate_bibi(experiment, target)
    simulation.cle = initialize_experiment(experiment, target)
