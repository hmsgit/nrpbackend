# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains the abstract base class of a simulation assembly
"""

import os
import sys
import random
import logging
from hbp_nrp_cleserver.server.ROSNotificator import ROSNotificator
from hbp_nrp_cleserver.bibi_config.notificator import Notificator

logger = logging.getLogger(__name__)


class SimulationAssembly(object):
    """
    This class is the abstract base class of a simulation assembly
    """

    def __init__(self, sim_config):
        """
        Creates a new simulation assembly

        :param sim_config: Simulation configuration
        """
        self._sim_config = sim_config

        self._abort_initialization = None
        self.ros_notificator = None
        self.rng_seed = None

    @property
    def sim_config(self):
        """
        Gets the sim_config
        """
        return self._sim_config

    @sim_config.setter
    def sim_config(self, sim_config):
        """
        Restrict external assignment of sim_config
        :raises AttributeError if the setter is called
        """
        # pylint: disable=no-self-use
        logger.info("Raising exception instead of assigning {} to sim_config".format(sim_config))
        raise AttributeError("sim_config has to be initialized on Assembly instantiation")

    # TODO: remove
    @property
    def simdir(self):
        """
        Gets the simulation directory
        """
        return self._sim_config.sim_dir

    @property
    def sim_dir(self):
        """
        Gets the simulation directory
        """
        return self._sim_config.sim_dir

    def initialize(self, except_hook):
        """
        Initializes the simulation
        :param environment: The environment that should be simulated
        :param except_hook: A method that should be called when there is a critical error
        """
        # Change working directory to experiment directory
        logger.info("Path is " + self._sim_config.sim_dir)
        os.chdir(self._sim_config.sim_dir)

        # RNG seed for components, use config value if specified or generate a new one
        self.rng_seed = self.sim_config.rng_seed
        if self.rng_seed is None:
            logger.info('No RNG seed specified in the exc, using random value.')
            self.rng_seed = random.randint(1, sys.maxint)
        logger.info('RNG seed = %i', self.rng_seed)

        # Create backend->frontend/client ROS notificator
        logger.info("Setting up backend Notificator")
        self.ros_notificator = ROSNotificator()
        Notificator.register_notification_function(
            lambda subtask, update_progress:
            self.ros_notificator.update_task(subtask, update_progress, True)
        )

        # Number of subtasks is not used in frontend notificator logic
        self.ros_notificator.start_task("Neurorobotics Platform",
                                        "Neurorobotics Platform",
                                        number_of_subtasks=1,
                                        block_ui=True)
        self._notify("Starting Neurorobotics Platform")

        self._initialize(except_hook)

        # Loading is completed.
        self._notify("Finished")
        self.ros_notificator.finish_task()

        logger.info("CLELauncher Finished.")

    def _initialize(self, except_hook):  # pragma: no cover
        """
        Internally initialize the simulation
        :param environment: The environment that should be simulated
        :param except_hook: A method that should be called when there is a critical error
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    def _notify(self, message):
        """
        Checks whether the simulation should abort immediately
        """
        if self._abort_initialization is not None:
            raise Exception("The simulation must abort due to: " + self._abort_initialization)
        Notificator.notify(message, True)

    def run(self):  # pragma: no cover
        """
        Runs the simulation
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    def shutdown(self):  # pragma: no cover
        """
        Shuts down the simulation
        """
        raise NotImplementedError("This method must be overridden in an implementation")
