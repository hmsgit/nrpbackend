"""
CLE states exposed via ROS.
This module implements the states and transitions of the simulation on the server side.
Every state is a class deriving from the State class and every valid transition from one state
is an implemented method the corresponding class. The other (invalid) transitions raise exceptions
by default.
"""


class ROSCLEState(object):
    """
    Describe the state that are exposed to the outside world through ROS.
    """
    STARTED = "started"
    STOPPED = "stopped"
    INITIALIZED = "initialized"
    PAUSED = "paused"
    FAILED = "failed"


class State(object):
    """
    Represents the state in which a ROSCLEServer instance can be.
    This is the base class defining the basic behavior, which means
    that no transitions are to be made to other states and this base
    state itself is not a final state.
    """
    def __init__(self, context):
        self._context = context

    # We disable the docstring here since there is nothing more to say than
    # what the method name already reveals.
    # pylint: disable=missing-docstring, unused-argument
    def reset_simulation(self, request):
        raise RuntimeError('You cannot reset the simulation while in %s.' %
                           (type(self).__name__, ))

    def stop_simulation(self):
        raise RuntimeError('You cannot stop the simulation while in %s.' %
                           (type(self).__name__, ))

    def pause_simulation(self):
        raise RuntimeError('You cannot pause the simulation while in %s.' %
                           (type(self).__name__, ))

    def start_simulation(self):
        raise RuntimeError('You cannot start the simulation while in %s.' %
                           (type(self).__name__, ))

    def fail(self):
        self._context.set_state(FailureState(self._context))

    # pylint: disable=no-self-use
    def is_final_state(self):
        return False


class FailureState(State):
    """
    The failure state means that the simulation failed and cannot be recovered
    """
    def is_final_state(self):
        return True

    def fail(self):
        pass

    def __repr__(self):
        return ROSCLEState.FAILED


class InitialState(State):
    """
    The initial state in which an instance of ROSCLEServer starts its lifecycle.
    """
    def start_simulation(self):
        result = self._context.start_simulation()
        self._context.set_state(RunningState(self._context))
        return result

    def stop_simulation(self):
        result = self._context.stop_simulation()
        self._context.set_state(StoppedState(self._context))
        return result

    def __repr__(self):
        return ROSCLEState.INITIALIZED


class RunningState(State):
    """
    Represents a running ROSCLEServer.
    """
    def reset_simulation(self, request):
        result = self._context.reset_simulation(request)
        self._context.set_state(InitialState(self._context))
        return result

    def stop_simulation(self):
        result = self._context.stop_simulation()
        self._context.set_state(StoppedState(self._context))
        return result

    def pause_simulation(self):
        result = self._context.pause_simulation()
        self._context.set_state(PausedState(self._context))
        return result

    def __repr__(self):
        return ROSCLEState.STARTED


class StoppedState(State):
    """
    Represents a stopped ROSCLEServer.
    """
    def is_final_state(self):
        return True

    def __repr__(self):
        return ROSCLEState.STOPPED


class PausedState(State):
    """
    Represents a paused ROSCLEServer.
    """
    def start_simulation(self):
        result = self._context.start_simulation()
        self._context.set_state(RunningState(self._context))
        return result

    def stop_simulation(self):
        result = self._context.stop_simulation()
        self._context.set_state(StoppedState(self._context))
        return result

    def reset_simulation(self, request):
        result = self._context.reset_simulation(request)
        self._context.set_state(InitialState(self._context))
        return result

    def __repr__(self):
        return ROSCLEState.PAUSED
