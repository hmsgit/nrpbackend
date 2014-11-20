"""
This module contains the REST services to setup the simulation
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations
from flask import request, url_for


@app.route('/simulation', methods=['POST'])
def setup_simulation():
    """
    Creates a new simulation. The simulation is not started
    """
    body = request.get_json(force=True)
    sim_id = len(simulations)
    if 'experimentID' in body:
        simulations.append({'experimentID': body['experimentID'], 'state': 'created'})
    else:
        return "Experiment ID is not valid", 400
    return "Simulation created successfully", 201, \
           {'location': url_for('set_simulation_state', sim_id=sim_id)}
