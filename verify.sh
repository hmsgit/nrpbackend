#!/bin/bash

source GazeboRosPackage/GazeboRosPackage/install/setup.bash

export IGNORE_LINT="platform_venv|hbp_nrp_backend/hbp_nrp_backend/bibi_config/generated|hbp_nrp_backend/hbp_nrp_backend/exd_config/generated|generated|build|CLE|hbp-flask-restful-swagger-master|GazeboRosPackage"
rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment_bibi.py

make verify
VERIFY_RET=$?

rm -rf GazeboRosPackage

exit $VERIFY_RET
