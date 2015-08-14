import mock
initialize_cb = None
def create_state_machine():
    return mock_instance

def __initialize_side_effect(*args, **kwargs):
    global initialize_cb
    initialize_cb = args[0]

mock_instance = mock.Mock()
mock_instance.sm.register_termination_cb.side_effect = __initialize_side_effect
