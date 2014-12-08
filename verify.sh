#!/bin/bash

module load boost/1.55zlib-rhel6-x86_64-gcc4.4
module load ros/hydro-rhel6-x86_64-gcc4.4
source $ROS_SETUP_FILE

virtualenv build_venv
. build_venv/bin/activate
pip install catkin_pkg

if [ ! -d CLE ]; then \
	git clone ssh://bbprelman@bbpcode.epfl.ch/neurorobotics/CLE
fi;\

# Installing the ros package to control gazebo
cd CLE/GazeboRosPackage/src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
# catkin_make install
cd ../..

deactivate

export IGNORE_LINT="platform_venv|generated|build|CLE|hbp-flask-restful-swagger-master|GazeboRosPackage"
rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment_bibi.py

make verify

