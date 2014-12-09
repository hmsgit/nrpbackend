"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'

from hbp_nrp_backend.exd_config import generate_experiment
from hbp_nrp_backend.simulation_control import simulations
import imp


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
    Resumes the simulation with the given simulation id
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
    target = '__generated_experiment.py'
    generate_experiment(experiment, target)

    # run script
    exd_script = imp.load_source('generated_exd', target)
    exd_script.initialize()
    simulation.cle = exd_script.cle
