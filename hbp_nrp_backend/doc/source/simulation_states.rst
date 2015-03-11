================================
State machine of the NRP Backend
================================

.. _state-machine:
.. figure:: img/stateMachine.png
    :align: center

    The state machine of the NRP Backend

The state machine of our current implementation of the NRP backend is depicted in :num:`Fig. #state-machine`. It shows the allowed transitions between states. The initial *created* state is reached after a *POST /simulation* request. Transitions between states are triggered by a *PUT /simulation/{id}/state* witht the new state as request body. See :doc:`REST-API` for details on requests to NRP Backend API.

The REST API */simulation/state* is implemented in class :class:`hbp_nrp_backend.rest_server.__SimulationState` and the state machine itself in class :class:`hbp_nrp_backend.simulation_control.__StateMachine`.

