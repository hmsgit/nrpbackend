#!/bin/bash

source /opt/rh/python27/enable
 
export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/modulefiles
export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/module/modulefiles
module load boost/1.55zlib-rhel6-x86_64-gcc4.4
 
module load ros/hydro-rhel6-x86_64-gcc4.4
source $ROS_SETUP_FILE
 
module load ros-hbp-packages/hydro-rhel6-x86_64-gcc4.4
source $ROS_HBP_PACKAGES_SETUP_FILE
 
export PYTHONPATH=/nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib/python2.7/site-packages:$PYTHONPATH
 
# OpenCV
module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2
# Gazebo
module load gazebo/4.0-rhel6-x86_64-gcc4.8.2
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64
export SDFormat_DIR=/nfs4/bbp.epfl.ch/sw/neurorobotics/sdf/2.0/rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib64/cmake/sdformat
export ESVRENDER_MATERIAL_PATH=/nfs4/bbp.epfl.ch/sw/neurorobotics/gazebo/4.0/rhel-6.5-x86_64/gcc-4.8.2/x86_64/share/gazebo-4.0
# Ogre (needed by Gazebo)
module load ogre/1.8.1-rhel6-x86_64-gcc4.8.2
# TBB
module load tbb/4.0.5-rhel6-x86_64-gcc4.4
# Additional libs
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/staging/lib64
 
module load nest/2.2.2
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib64/compat-openmpi/lib
export NRP_MODELS_DIRECTORY=/root/.gazebo/hbp