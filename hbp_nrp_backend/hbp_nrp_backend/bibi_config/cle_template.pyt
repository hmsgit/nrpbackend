# -*- coding: utf-8 -*-
"""
This file is generated. The original demo script this file is based on has been created by Lorenzo Vannucci
meanwhile the Template has been created by Georg Hinkel.
"""
# pragma: no cover

__author__ = 'BIBI Configuration Script'

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from os.path import expanduser
import os


def cle_function(world_file):

    from hbp_nrp_cle.cle.ROSCLEServer import ROSCLEServer

    # Create ROS server
    cle_server = ROSCLEServer()
    update_progress_function = lambda subtask, update_progress: cle_server.notify_current_task(subtask, update_progress, True)

    cle_server.notify_start_task("Initializing the Neurorobotic Close Loop Engine",
                              "Importing needed packages",
                              5, # number of subtasks
                              True)  # block_ui

    from hbp_nrp_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine
    
    from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_gazebo_model_file, empty_gazebo_world, load_gazebo_world_file
    from hbp_nrp_cle.robotsim.RobotInterface import Topic
    from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
    from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

    from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
    from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter

    import hbp_nrp_cle.tf_framework as nrp

    # Needed in order to cleanup global static variables
    nrp.start_new_tf_manager()

    # import dependencies from BIBI configuration
    {% for dep in dependencies %}
    import {{dep[:dep.rfind('.')]}} #import {{dep.split('.')|last()}}
    {% endfor %}

    # import transfer functions specified in Python
    {% if len(config.transferFunctionImport) > 0 %}# pylint: disable=W0401
    {% for imp in config.transferFunctionImport %}
    import {{remove_extension(imp)}}{% endfor %}{% endif %}

    {% for tf in config.transferFunction %}{% if tf.extensiontype_ == 'Neuron2Robot' %}
    {% for topic in tf.topic %}{% if is_not_none(topic.body) %}
    @nrp.MapRobotPublisher("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% else %}
    @nrp.MapRobotSubscriber("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endif %}{% endfor %}{% for dev in tf.device %}{% if is_not_none(dev.body) %}
    @nrp.MapSpikeSource("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% else %}
    @nrp.MapSpikeSink("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endif %}{% endfor %}
    @nrp.Neuron2Robot({% if is_not_none(tf.returnValue) %}Topic('{{tf.returnValue.topic}}', {{tf.returnValue.type_}}){% endif %})
    def {{tf.name}}(t{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
        {% for local in tf.local %}{{local.name}} = {{print_expression(local.body)}}
        {% endfor %}
        {% for dev in tf.device %}{% if is_not_none(dev.body) %}{{dev.name}}.{{get_default_property(dev.type_)}} = {{print_expression(dev.body)}}
        {% endif %}{% endfor %}
        {% for top in tf.topic %}{% if is_not_none(top.body) %}{{top.name}}.send_message({{print_expression(top.body)}})
        {% endif %}{% endfor %}{% if is_not_none(tf.topic.body) %}
        {% if is_not_none(tf.returnValue) %}return {{print_expression(tf.returnValue.body)}}{% endif %}{% endif %}

    {% else %}{% for topic in tf.topic %}{% if is_not_none(topic.body) %}
    @nrp.MapRobotPublisher("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% else %}
    @nrp.MapRobotSubscriber("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endif %}{% endfor %}{% for dev in tf.device %}{% if is_not_none(dev.body) %}
    @nrp.MapSpikeSource("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% else %}
    @nrp.MapSpikeSink("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endif %}{% endfor %}
    @nrp.Robot2Neuron()
    def {{tf.name}}(t{% for topic in tf.topic %}, {{topic.name}}{%endfor%}{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
        {% for local in tf.local %}{{local.name}} = {{print_expression(local.body)}}
        {% endfor %}
        {% for dev in tf.device %}{% if is_not_none(dev.body) %}{{dev.name}}.{{get_default_property(dev.type_)}} = {{print_expression(dev.body)}}
        {% endif %}{% endfor %}
        {% for top in tf.topic %}{% if is_not_none(top.body) %}{{top.name}}.send_message({{print_expression(top.body)}})
        {% endif %}{% endfor %}{% endif %}{% endfor %}


    # consts
    TIMESTEP = 0.01
    MAX_SIM_TIME = 5

    update_progress_function("Reseting Gazebo robotic simulator", True)
    empty_gazebo_world(update_progress_function)

    cle_server.notify_current_task("Loading experiment environment",
                                True,  # update_progress
                                True)  # block_ui
    load_gazebo_world_file(world_file, update_progress_function)

    # Create interfaces to Gazebo
    cle_server.notify_current_task("Loading neuRobot",
                                True,  # update_progress
                                True)  # block_ui
    # spawn robot model
    load_gazebo_model_file('robot', '{{config.bodyModel}}')

    # control adapter
    roscontrol = RosControlAdapter()
    # communication adapter
    roscomm = RosCommunicationAdapter()


    # Create interfaces to brain
    cle_server.notify_current_task("Loading neural Simulator NEST",
                                True,  # update_progress
                                True)  # block_ui
    # control adapter
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    brainfilepath = '{{config.brainModel.file}}'
    if models_path is not None:
        brainfilepath = os.path.join(models_path, brainfilepath)
    braincontrol = PyNNControlAdapter(brainfilepath{% for p in config.brainModel.neuronGroup %},
                                      {{p.population}}={{print_neurons(p)}}{% endfor %})
    # communication adapter
    braincomm = PyNNCommunicationAdapter()


    # Create transfer functions manager
    cle_server.notify_current_task("Connecting neural simulator to neurobot",
                                True,  # update_progress
                                True)  # block_ui
    # tf manager
    tfmanager = nrp.config.active_node
    # set adapters
    tfmanager.robot_adapter = roscomm
    tfmanager.brain_adapter = braincomm


    # Create CLE
    cle = SerialClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)
    # initialize everything
    cle.initialize()

    # Now that we have everything ready, we could prepare the simulation
    cle_server.prepare_simulation(cle)
    # Loading is completed.
    cle_server.notify_finish_task()
    
    # Main infinite loop (until the ROS stop service is called)
    cle_server.main()

    # Once we do reach this point, the simulation is stopped and we could clean after ourselves.
    cle_server.shutdown()