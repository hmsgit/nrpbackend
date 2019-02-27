#!/bin/bash
# This script is designed for local usage.

# Code coverage does not seem work without /nfs4 or /gpfs access
export VIRTUAL_ENV=$NRP_VIRTUAL_ENV
export IGNORE_LINT="Experiments/|Models/|platform_venv|ExperimentControl|CLE|BrainSimulation|hbp_nrp_commons/hbp_nrp_commons/generated|hbp-flask-restful-swagger-master|GazeboRosPackages|migrations|nest"

make test
RET=$?

~/.opt/platform_venv/bin/coverage html  --omit '~/.opt/platform_venv/*,./*/tests/*,hbp-flask-restful-swagger-master/*,hbp_nrp_commons/hbp_nrp_commons/generated/*' --include "$(pwd)/*"
~/.opt/platform_venv/bin/coverage report  --omit '~/.opt/platform_venv/*,./*/tests/*,hbp-flask-restful-swagger-master/*,hbp_nrp_commons/hbp_nrp_commons/generated/*' --include "$(pwd)/*" -m

if [ $RET == 0 ]; then
    echo -e "\033[32mTest sucessfull.\e[0m"
else
    echo -e "\033[31mTest failed.\e[0m See errors above."
fi

exit $RET
