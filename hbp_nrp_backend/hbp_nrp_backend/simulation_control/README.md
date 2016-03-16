# Simulation control

The simulation can be controlled by changing its state. The possible
state transitions are:

created
    --> initialized
        loads the environment, the robot model, the neural model
        and wires the transfer functions

initialized
    --> started
        starts the simulation
    --> stopped
        stops the simulation and release the resources

started
    --> paused
        pauses the simulation
    --> initialized
        resets the simulation to its inital state
    --> stopped
        stops the simulation and release the resources

paused
    --> started
        restarts the simulation
    --> initialized
        resets the simulation to its inital state
    --> stopped
        stops the simulation and release the resources
