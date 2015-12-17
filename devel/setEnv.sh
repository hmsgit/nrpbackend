#!/usr/bin/env bash

export HBP=$( cd "$( dirname "${BASH_SOURCE[0]}")/../../" && pwd )
export CLE=$HBP/CLE
export EXC=$HBP/ExperimentControl
export EXDB=$HBP/ExDBackend
export EXDF=$HBP/ExDFrontend
export GZ_ROS_PKGS=$HBP/GazeboRosPackages
export NRP_MODELS_DIRECTORY=$HBP/Models
source /opt/ros/indigo/setup.bash
source $GZ_ROS_PKGS/devel/setup.bash
export PATH=$PATH:$EXDB/devel
export PYTHONPATH=$PYTHONPATH:/opt/nest/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/indigo/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$GZ_ROS_PKGS/devel/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$EXC/hbp_nrp_excontrol
export PYTHONPATH=$PYTHONPATH:$EXDB/hbp_nrp_backend
export PYTHONPATH=$PYTHONPATH:$EXDB/hbp_nrp_cleserver
export PYTHONPATH=$PYTHONPATH:$EXDB/hbp_nrp_commons
export PYTHONPATH=$PYTHONPATH:$EXDB/platform_venv/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$CLE/hbp_nrp_cle
export PYTHONPATH=$PYTHONPATH:$CLE/platform_venv/lib/python2.7/site-packages

export GAZEBO_MODELS=(husky_model library_model terrain_models virtual_room hosta_potted_plant lamp_models/vr_lamp maze_models mouse_model poster_models/viz_poster poster_models/viz_poster_2 tree_models vr_screen )

