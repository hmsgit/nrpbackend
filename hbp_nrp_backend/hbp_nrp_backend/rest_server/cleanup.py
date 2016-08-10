"""
This module defines a job that runs in the background and looks for expired simulations
"""

import datetime
import time
from hbp_nrp_backend.simulation_control import simulations, timezone
import logging

__author__ = 'Georg Hinkel'


logger = logging.getLogger(__name__)


def clean_simulations():  # pragma: no cover
    """
    Wakes up every ten minutes and stops expired simulations

    This function is not unit-tested as it contains an endless loop
    """
    while True:
        time.sleep(600)
        logger.info("Start cleanup")
        current_time = datetime.datetime.now(timezone)
        for sim in simulations:
            if sim.kill_datetime is not None and sim.kill_datetime < current_time:
                logger.info("Stopping expired simulation " + str(sim.sim_id))
                sim.state = 'stopped'
