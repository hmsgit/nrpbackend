"""
This module contains the REST implementation for the simulation control
"""

__author__ = 'GeorgHinkel'

from flask import request, jsonify

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import stateMachine, simulations


@app.route('/simulation/<int:sim_id>/state', methods=['PUT', 'GET'])
def set_simulation_state(sim_id):
    """
    Sets the state of the simulation with the given id
    :param sim_id:The simulation id
    """
    if sim_id < 0 or sim_id >= len(simulations):
        return "Simulation does not exist", 404
    simulation = simulations[sim_id]
    if request.method == 'PUT':
        body = request.get_json(force=True)
        new_state = body['state']
        current_state = simulation['state']
        action = stateMachine[current_state].get(new_state)
        if action is None:
            return "Transition is invalid", 400
        simulation['state'] = new_state
        action(sim_id)
    return jsonify(simulation)


@app.route('/simulation/<int:sim_id>/events/<event_id>', methods=['PUT'])
def raise_hardware_event(sim_id, event_id):
    """
    Raises a hardware event
    :param sim_id: The simulation id
    :param event_id: The event id
    """
    body = request.get_json(force=True)
    value = body['value']
    return "Setting hardware event " + event_id + " for simulation " +\
           sim_id + " to value " + value.__repr__()
