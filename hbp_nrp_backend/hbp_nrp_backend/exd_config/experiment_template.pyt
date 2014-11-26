"""
Experiment launcher
This file is generated.
"""
# pragma: no cover

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from os.path import expanduser
import imp

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

__author__ = 'ExD Configuration Script'

cle = None

def start():
    spawn_gazebo_sdf('environment', '{{model}}')
    bibi_script = imp.load_source('bibi_configuration', "{{bibi_script}}")
    global cle
    cle = bibi_script.cle