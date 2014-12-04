"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'

# import os
# from hbp_nrp_backend.exd_config import generate_experiment
# from hbp_nrp_backend.simulation_control import simulations
# import imp


def start_simulation(sim_id):
    """
    Starts the simulation with the given id
    :param sim_id: The simulation id
    """
#    simulation = simulations[sim_id]
#    directory = os.path.split(__file__)[0]
#    experiment = simulation.experiment_id
#    target = os.path.join(directory, '__generate_experiment.py')
#    generate_experiment(experiment, target)

#    experiment_start = imp.load_source('experiment.setup', target)
#    experiment_start.start()
#    simulation.cle = experiment_start.cle

    print "Start simulation " + str(sim_id)


def pause_simulation(sim_id):
    """
    Pauses the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Pause simulation " + str(sim_id)


def reset_simulation(sim_id):
    """
    Resumes the simulation with the given simulation id
    :param sim_id: The simulation id
    """
    print "Reset simulation " + str(sim_id)


def stop_simulation(sim_id):
    """
    Stops the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Stop simulation " + str(sim_id)


def initialize_simulation(sim_id):
    """
    Releases the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Initialize simulation " + str(sim_id)
