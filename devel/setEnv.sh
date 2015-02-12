
export HBP=$( cd "$( dirname "${BASH_SOURCE[0]}")/../../" && pwd )
export CLE=$HBP/CLE
export EXDB=$HBP/ExDBackend
export EXDF=$HBP/ExDFrontend
export NRP_MODELS_DIRECTORY=$HBP/Models
source /opt/ros/indigo/setup.bash
source $CLE/GazeboRosPackage/devel/setup.bash
export PATH=$PATH:$EXDB/etc
export PYTHONPATH=$PYTHONPATH:/opt/nest/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/indigo/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$HBP/CLE/GazeboRosPackage/devel/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$EXDB/hbp_nrp_backend
export PYTHONPATH=$PYTHONPATH:$CLE/hbp_nrp_cle
export GAZEBO_MODELS=(husky_model library_model terrain_models virtual_room hosta_potted_plant lamp_models/vr_lamp maze_models mouse_model poster_models/viz_poster poster_models/viz_poster_2 tree_models vr_screen )

