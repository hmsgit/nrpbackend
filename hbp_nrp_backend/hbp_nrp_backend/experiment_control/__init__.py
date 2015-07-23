'''
This package manages the state machines which control the experiment and decide whether it
succeeded or failed
'''
__author__ = 'Sebastian Krach'

MAX_TIMEOUT = 5


class TimeoutException(Exception):
    """
    The exception which is thrown for blocking calls when a time out occurs.
    """

    def __init__(self, message):
        """
        Initializes a new timeout exception instance.

        :param message: A textual description of the situation where the timeout occured.
        """

        super(TimeoutException, self).__init__(message)


class ConcurrentExecutionException(Exception):
    """
    The exception which is thrown whenever a user tries to modify a running state machine
    """

    def __init__(self, message):
        """
        Initializes a new concurrent execution exception instance.

        :param message: A textual description of the situation leading to the problem.
        """

        super(ConcurrentExecutionException, self).__init__(message)


from hbp_nrp_backend.experiment_control.__ExperimentStateMachine \
    import ExperimentStateMachineInstance
