"""
This file loads the production transitions
"""


__author__ = 'GeorgHinkel'


def start_simulation(sim_id):
    """
    Starts the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Start simulation" + str(sim_id)


def pause_simulation(sim_id):
    """
    Pauses the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Pause simulation" + str(sim_id)


def resume_simulation(sim_id):
    """
    Resumes the simulation with the given simulation id
    :param sim_id: The simulation id
    """
    print "Resume simulation" + str(sim_id)


def stop_simulation(sim_id):
    """
    Stops the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Stop simulation" + str(sim_id)


def release_simulation(sim_id):
    """
    Releases the simulation with the given id
    :param sim_id: The simulation id
    """
    print "Release simulation" + str(sim_id)
