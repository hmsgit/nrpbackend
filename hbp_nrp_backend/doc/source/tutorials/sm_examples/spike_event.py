def condition_spike_cb(user_data, spike):
    # ... parse JSON in spike message ...
    # return TRUE as long as condition is not met
    return spike < 10


if __name__ == "__main__":
    # ...

        smach.StateMachine.add('CONDITION',
                               smach_ros.MonitorState('/monitoring/left_wheel_neuron_rate_monitor',
                                                      String,
                                                      condition_spike_cb),
                               {'valid': 'CONDITION', 'invalid': 'ACTION',
                                'preempted': 'CONDITION_PREEMPTED'})
