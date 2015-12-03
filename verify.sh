#!/bin/bash

# ------------------------------------------------------------------
# Simple wrapper around the "make verify" target of the common hbp 
# Makefile for python. This is mainly use for local development.
# ------------------------------------------------------------------

read -d '' USAGE << EOF
Usage: ./verify.sh -hr\n
r     Try to release the code to HBP pypy server\n
h     Print usage

EOF

while getopts ":rh" optname
  do
    case "$optname" in
      "r")
        RELEASE=true
        ;;
      "h")
        echo "$USAGE"
        exit 0;
        ;;
      "?")
        echo "Unknown option $OPTARG"
        echo "$USAGE"
        exit 0;
        ;;
      *)
        echo "Unknown error while processing options"
        exit 0;
        ;;
    esac
  done

export IGNORE_LINT="platform_venv|hbp_nrp_backend/hbp_nrp_backend/exd_config/generated|hbp_nrp_commons/hbp_nrp_commons/generated|generated|build|CLE|hbp-flask-restful-swagger-master|GazeboRosPackage|hbp_nrp_backend/hbp_nrp_backend/rest_server/migrations"
rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment_bibi.py

if [ "$RELEASE" = true ] ; then
    make verify
else
	make verify_base
fi
VERIFY_RET=$?

exit $VERIFY_RET
