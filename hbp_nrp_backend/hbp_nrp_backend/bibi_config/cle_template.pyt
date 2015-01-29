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

def cle_function():

    from hbp_nrp_cle.cle.ROSCLEWrapper import ROSCLEServer
    from hbp_nrp_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine
    
    from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_gazebo_model_file
    from hbp_nrp_cle.robotsim.RobotInterface import Topic
    from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
    from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

    from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
    from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter

    import hbp_nrp_cle.tf_framework as nrp

    # import dependencies from BIBI configuration
    {% for dep in dependencies %}
    import {{dep[:dep.rfind('.')]}} #import {{dep.split('.')|last()}}
    {% endfor %}

    # import transfer functions specified in Python
    {% if len(config.transferFunctionImport) > 0 %}# pylint: disable=W0401
    {% for imp in config.transferFunctionImport %}
    import {{remove_extension(imp)}}{% endfor %}{% endif %}

    {% for tf in config.transferFunction %}{% if tf.extensiontype_ == 'Neuron2Robot' %}
    {% for topic in tf.topic %}{% if is_not_none(tf.body) %}
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

    {% else %}{% for topic in tf.topic %}{% if is_not_none(tf.body) %}
    @nrp.MapRobotSubscriber("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% else %}
    @nrp.MapRobotPublisher("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endif %}{% endfor %}{% for dev in tf.device %}{% if is_not_none(dev.body) %}
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

    import signal

    # consts
    TIMESTEP = 0.01
    MAX_SIM_TIME = 5

    # SIGINT handler
    flag = False
    def handler(signum, frame):
        global flag
        flag = True

    signal.signal(signal.SIGINT, handler)

    # Create interfaces to Gazebo

    # spawn robot model
    load_gazebo_model_file('robot', '{{config.bodyModel}}')

    # control adapter
    roscontrol = RosControlAdapter()
    # communication adapter
    roscomm = RosCommunicationAdapter()


    # Create interfaces to brain

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

    # tf manager
    tfmanager = nrp.config.active_node
    #assert isinstance(tfmanager, _TransferFunctionManager.TransferFunctionManager)
    # set adapters
    tfmanager.robot_adapter = roscomm
    tfmanager.brain_adapter = braincomm


    # Create CLE
    cle = SerialClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)
    # initialize everything
    cle.initialize()

    # Create ROS Wrapper
    wrapper = ROSCLEServer(cle)
    wrapper.main()
