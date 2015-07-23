"""
This package contains the ExD Configuration script
"""

__author__ = 'Lorenzo Vannucci'

OUTCOME_FINISHED = 'FINISHED'
OUTCOME_ERROR = 'ERROR'
OUTCOME_CONDITION_PREEMPTED = 'CONDITION_PREEMPTED'
OUTCOME_ACTION_PREEMPTED = 'ACTION_PREEMPTED'
OUTCOME_ACTION_ERROR = 'ACTION_ERROR'

OUTCOMES_SUCCESS = [OUTCOME_FINISHED]
OUTCOMES_FAILURE = [OUTCOME_ACTION_ERROR, OUTCOME_ERROR]
OUTCOMES_EARLY_TERMINATION = [OUTCOME_ACTION_PREEMPTED, OUTCOME_CONDITION_PREEMPTED]

OUTCOMES = [OUTCOME_FINISHED, OUTCOME_ACTION_ERROR, OUTCOME_ERROR,
            OUTCOME_ACTION_PREEMPTED, OUTCOME_CONDITION_PREEMPTED]

from hbp_nrp_backend.exd_config.experiment_configuration_script import generate_bibi
from hbp_nrp_backend.exd_config.experiment_configuration_script import generate_experiment_control
from hbp_nrp_backend.exd_config.experiment_configuration_script import initialize_experiment
