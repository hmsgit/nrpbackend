#!/bin/bash

export IGNORE_LINT="platform_venv|generated|build"
rm hbp_nrp_backend/hbp_nrp_backend/bibi_config/tests/generated_cle_script.py

make verify

