Tutorial: Writing a state machine for an experiment
===================================================

The NRP is designed to use state machines to control and evaluate experiments. A state machine controlling an experiment can monitor any simulation properties published on ROS topics (e.g. simulation time, sensor output, spiking activity of brain) and publish on ROS topics or call ROS services. A state machine evaluating the success of an experiment has the same capabilities for monitoring a simulation and should either publish a success or failure message to the NRP ROS status topic.

The NRP uses SMACH for executing state machines, hence state machines have to be specified in Python.


Controlling an experiment
-------------------------

The following examples show how to specify state machines for certain tasks.

Trigger an action after 30 seconds of simulation time
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/time_event.py
   :language: python

This example contains two states: a `MonitorState <http://wiki.ros.org/smach/Tutorials/MonitorState>`_ monitoring the '/ros_cle_simulation/status' ROS topic as condition and a `ServiceSate <http://wiki.ros.org/smach/Tutorials/ServiceState>`_ calling the '/gazebo/set_visual_properties' ROS service to change the screen color as action. The MonitorState executes a callback each time a message is received on the topic.


Trigger an action based on user interaction
"""""""""""""""""""""""""""""""""""""""""""

.. note::
    Currently, the ExD back-end directly executes ROS services upon receiving user interactions/events. In the future the name of the user interaction should be published on a ROS topic to allow the control state machine to execute the user interaction/event.


Trigger an action based on a monitored property
"""""""""""""""""""""""""""""""""""""""""""""""

The MonitorState allows to monitor any available ROS topic. Thus, monitoring a property can be done in the same way as for simulation time.

.. literalinclude:: sm_examples/spike_event.py
   :language: python

In this example a neuron monitor, set up in the according BIBI configuration file, is used to monitor brain activity and use it as condition for an event.

Client Logging
--------------

'Client logs' is a logging mechanism that aims at helping the users develop and debug their state machines and transfer functions.
All triggered client log will be visible by the user in a specific window from within the running simulation.

Triggering client logs
""""""""""""""""""""""

The clientLogger allows the user to trigger client logs to help debugging  his state machines.

.. literalinclude:: sm_examples/client_logger.py
   :language: python

In this example, we are interested in having a state machine that will change state when the robot is at the right or left limit of the scene.
We know that the RobotPoseMonitorState allows us to have a condition on the robot coordinates (x,y,z), but we don't know exactly what values correspond to positions we want to watch for.
By using the clientLogger in that function watching the robot position, we can log the (x,y,z) values as we drive the robot in the scene to the target positions.
We can then collect the values at the target position and use them in the state condition.


State client logger
"""""""""""""""""""

The ClientLogState is a state that trigger a client log.

.. literalinclude:: sm_examples/state_client_logger.py
   :language: python

In this example, we are using the positions we collected in the section above and we expect the state machine to change state as the robot position alternates from left to right.
To test the validity of our parameters, we've added the ClientLogState("Husky is at the LEFT/RIGHT!") states that will log messages as the state machines transitions from one state to the other.
By moving the robot around and observing the log messages ("Husky is at the LEFT/RIGHT!") appear in the simulation, the user can verify that the transitions occur at the expected robot position.

Trigger an action based on a monitored client log
"""""""""""""""""""""""""""""""""""""""""""""""

The WaitForClientLogState allows to monitor for a specific client log.
The action is triggered when a log message contains the given string. The check is case sensitive. 

.. literalinclude:: sm_examples/client_logger_watcher.py
   :language: python

In this example, we want the state machine to change its state when a specific client log containing the keywords 'left_tv_red' or 'left_tv_blue' is observed.

Combined events
---------------

The following examples show how to combine several events into a single state machine.

Multiple time events as sequence
""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_time_events.py
   :language: python

In this example the first event is triggered after 30 seconds and the second one after 60 seconds. The actions of both events are implemented by a `CBState <http://wiki.ros.org/smach/Tutorials/CBState>`_, simply executing a callback.


Multiple conditions for a single event
""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_conditions_event.py
   :language: python

This example shows how to combine multiple conditions for a single event by using a `Concurrence Container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. Both conditions, simulation time larger than 30 seconds and brain activity above threshold, have to be met. The order the conditions become true doesn't matter.


Multiple independent events in a single state machine
"""""""""""""""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: sm_examples/multiple_events_sm.py
   :language: python

This example shows how to combine multiple events in a single state machine by nesting `State Machine Containers <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`_ (events) into a `Concurrence Container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`_. The concurrence container terminates a soon as both nested state machines (events) terminate.


Evaluating experiment result
----------------------------

A state machine for evaluating the experiment success does not differ from an experiment control state machine in terms of capabilities. However, it should have exactly two final states: one publishing a 'success' message on the CLE status topic and one publishing a 'failure' message.

.. note::
    Currently, the frontend is not able to visualize success or failure messages.

