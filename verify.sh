#!/bin/bash

export IGNORE_LINT="platform_venv|generated|build"
rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment.py
rm hbp_nrp_backend/hbp_nrp_backend/exd_config/tests/experiment_bibi.py

make verify

