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


def spawn_gazebo_sdf(model_name, model_file, initial_pose=None):
    """
    Generates Code to run the experiment based on the given configuration file
    :param model_name: Name of the model inside the gazebo scene
    :param model_file: The name of the model inside the NRP_MODELS_DIRECTORY \
        folder. If the NRP_MODELS_DIRECTORY environment variable is not set, \
        this script will search the model in its own folder.
    :param initial_pose: Initial pose of the model. Uses the Gazebo \
        "Pose" type.
    """

    # find & open sdf file
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    if models_path is None:
        models_path = os.path.dirname(__file__)
    mdl = open(os.path.join(models_path, model_file), 'r')
    sdff = mdl.read()
    mdl.close()

    # set initial pose
    if initial_pose is None:
        initial_pose = Pose()
        initial_pose.position = Point(0, 0, 0)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)

    # spawn model
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(model_name,
                     sdff,
                     "",
                     initial_pose,
                     "")
    spawn_model_prox.close()


def cle_function():

    from hbp_nrp_cle.cle.ROSCLEWrapper import ROSCLEServer
    from hbp_nrp_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine

    from hbp_nrp_cle.robotsim.RobotInterface import Topic
    from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
    from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

    from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
    from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter

    import hbp_nrp_cle.tf_framework as nrp

    # import dependencies from BIBI configuration
    
    import geometry_msgs.msg #import Twist
    
    import hbp_nrp_cle.tf_framework.tf_lib #import detect_red
    
    import sensor_msgs.msg #import Image
    

    # import transfer functions specified in Python
    

    
    
    @nrp.MapRobotPublisher("wheel", Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[2], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot()
    def linear_twist(t, left_wheel_neuron, right_wheel_neuron):
        
        
        wheel.send_message(geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=500.0 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage), y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1500.0 * (left_wheel_neuron.voltage - right_wheel_neuron.voltage))))
        
        

    
    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[0], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[1], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[2], nrp.poisson)
    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):
        image_results = hbp_nrp_cle.tf_framework.tf_lib.detect_red(image=camera.value)
        
        red_left_eye.rate = 0.002 * image_results.left
        red_right_eye.rate = 0.002 * image_results.right
        green_blue_eye.rate = 0.00025 * image_results.go_on
        
        

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
    spawn_gazebo_sdf('robot', 'husky.sdf')

    # control adapter
    roscontrol = RosControlAdapter()
    # communication adapter
    roscomm = RosCommunicationAdapter()


    # Create interfaces to brain

    # control adapter
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')
    brainfilepath = 'None'
    if models_path is not None:
        brainfilepath = os.path.join(models_path, brainfilepath)
    braincontrol = PyNNControlAdapter(brainfilepath)
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