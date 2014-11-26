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

def spawn_gazebo_sdf(model_name, model_file):
    """
    Generates Code to run the experiment based on the given configuration file
    :param model_name: Name of the model inside the gazebo scene
    :param model_file: The name of the model inside the ~/.gazebo/models folder
    """

    # find & open sdf file
    home = expanduser("~")
    filepath = home + '/.gazebo/models/' + model_file + '/model.sdf'
    mdl = open(filepath, 'r')
    sdff = mdl.read()
    mdl.close()

    # set initial pose
    initial_pose = Pose()
    initial_pose.position = Point(0, 0, 0)
    initial_pose.orientation = Quaternion(0, 0, 0, 1)

    # spawn model
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(model_name,
                     sdff,
                     "",
                     initial_pose,
                     "")
    spawn_model_prox.close()

from python_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine

from python_cle.brainsim import BrainLoader
from python_cle.robotsim.RobotInterface import Topic
from python_cle.robotsim.RosControlAdapter import RosControlAdapter
from python_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

from python_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
from python_cle.brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter

import python_cle.tf_framework as nrp

# import dependencies from BIBI configuration
{% for dep in dependencies %}
import {{dep[:dep.rfind('.')]}} #import {{dep.split('.')|last()}}
{% endfor %}

# import transfer functions specified in Python
{% if len(config.transferFunctionImport) > 0 %}# pylint: disable=W0401
{% for imp in config.transferFunctionImport %}
import {{remove_extension(imp)}}{% endfor %}{% endif %}

{% for tf in config.transferFunction %}{% if tf.extensiontype_ == 'Neuron2Robot' %}
{% for dev in tf.device %}
@nrp.MapNeuronParameter("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endfor %}
@nrp.Neuron2Robot(Topic('{{tf.topic.topic}}', {{tf.topic.type_}}))
def {{tf.name}}(t{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
    {% for local in tf.local %}{{local.name}} = {{print_expression(local.body)}}
    {% endfor %}
    return {{print_expression(tf.topic.body)}}

{% else %}{% for topic in tf.topic %}
@nrp.MapRobotParameter("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endfor %}{% for dev in tf.device %}
@nrp.MapNeuronParameter("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endfor %}
@nrp.Robot2Neuron()
def {{tf.name}}(t{% for topic in tf.topic %}, {{topic.name}}{%endfor%}{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
    {% for local in tf.local %}{{local.name}} = {{print_expression(local.body)}}
    {% endfor %}
    {% for dev in tf.device %}{{dev.name}}.{{get_default_property(dev.type_)}} = {{print_expression(dev.body)}}
    {% endfor %}{% endif %}{% endfor %}

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
spawn_gazebo_sdf('robot', '{{config.bodyModel}}')

# control adapter
roscontrol = RosControlAdapter()
# communication adapter
roscomm = RosCommunicationAdapter()

# Create interfaces to brain

# control adapter
braincontrol = PyNNControlAdapter()
BrainLoader.load_h5_network('{{config.brainModel}}', [0, 1, 2], [3, 4, 5])
import PyNNScript
PyNNScript.init_brain_simulation()
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
cle = SerialClosedLoopEngine(roscontrol, braincontrol, tfmanager, TIMESTEP)
# initialize everything
cle.initialize()