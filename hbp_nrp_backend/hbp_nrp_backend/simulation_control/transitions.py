"""
This file loads the production transitions
"""


__author__ = 'Georg Hinkel'

from hbp_nrp_backend.exd_config \
    import initialize_experiment, generate_experiment_control
from hbp_nrp_backend import NRPServicesGeneralException
from flask import request
from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
from gazebo_msgs.srv import SetVisualProperties
from cle_ros_msgs.srv import ResetSimulationRequest
from hbp_nrp_backend.simulation_control.__SimulationCLEHandler import SimulationCLEHandler

import os
import shutil
import logging
import rospy
import tempfile

logger = logging.getLogger(__name__)


def start_simulation(simulation):
    """
    Starts the given simulation

    :param simulation: The simulation
    """
    if simulation.state != 'paused':
        logger.info("starting State Machines...")
        try:
            simulation.state_machine_manager.start_all()
        # pylint: disable=bare-except
        except:
            logger.error("Starting State Machines Failed")
            # TODO: how to report this error to the front-end?

    logger.info("starting CLE...")
    simulation.cle.start()

    logger.info("simulation started")


def pause_simulation(simulation):
    """
    Pauses the simulation

    :param simulation: The simulation
    """
    simulation.cle.pause()
    logger.info("simulation paused")


def reset_simulation(simulation):
    """
    Reset the given simulation

    :param simulation: The simulation
    """

    logger.info("State machine outcomes: %s", ", ".join("%s: %s" % (sm.sm_id, str(sm.result))
                                                        for sm in simulation.state_machines))
    simulation.state_machine_manager.terminate_all()

    # The following two lines are part of a fix for [NRRPLT-1899]
    # To be removed when the following Gazebo issue is solved:
    # https://bitbucket.org/osrf/gazebo/issue/1573/scene_info-does-not-reflect-older-changes
    simulation.left_screen_color = 'Gazebo/Blue'  # pragma: no cover
    simulation.right_screen_color = 'Gazebo/Blue'  # pragma: no cover
    simulation.cle.reset(ResetSimulationRequest.RESET_OLD)

    # The following lines are part of a fix for [NRRPLT-2844]
    # As a Gazebo reset does not touch visual properties, we have to reset them explicitly
    try:
        set_visual_properties = rospy.ServiceProxy('/gazebo/set_visual_properties',
                                                   SetVisualProperties)
        set_visual_properties(model_name='right_vr_screen', link_name='body',
                              visual_name='screen_glass',
                              property_name='material:script:name',
                              property_value='Gazebo/Blue')
        set_visual_properties(model_name='left_vr_screen', link_name='body',
                              visual_name='screen_glass',
                              property_name='material:script:name',
                              property_value='Gazebo/Blue')
    except rospy.ServiceException:
        pass

    try:
        simulation.state_machine_manager.start_all()
    # pylint: disable=bare-except
    except:
        logger.error("Starting State Machines Failed")
        # TODO: how to report this error to the front-end?
    logger.info("simulation reset")


def stop_simulation(simulation):
    """
    Stops the given simulation

    :param simulation: The simulation
    """
    simulation.cle.stop()

    logger.info("State machine outcomes: %s", ", ".join("%s: %s" % (sm.sm_id, str(sm.result))
                                                        for sm in simulation.state_machines))

    simulation.state_machine_manager.terminate_all()

    logger.info("simulation stopped")


def initialize_simulation(simulation):
    """
    Releases the given simulation

    :param simulation: The simulation
    """
    # generate script
    try:
        # Done here in order to avoid circular dependencies introduced by the
        # way we __init__ the rest_server module.
        from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient \
            import NeuroroboticsCollabClient
        models_path = os.environ.get('NRP_MODELS_DIRECTORY')
        using_collab_storage = simulation.context_id is not None
        if using_collab_storage:
            client = NeuroroboticsCollabClient(
                UserAuthentication.get_header_token(request), simulation.context_id)
            experiment_path = client.clone_experiment_template_from_collab_context()
            experiment = experiment_path['experiment_conf']
            environment = experiment_path['environment_conf']
        else:
            experiment = os.path.join(models_path, simulation.experiment_conf)
            environment = simulation.environment_conf

        gzserver_host = simulation.gzserver_host
        state_machine_paths = generate_experiment_control(experiment, models_path)
        simulation.state_machine_manager.add_all(state_machine_paths, simulation.sim_id)
        simulation.state_machine_manager.initialize_all()

        simulation.cle = initialize_experiment(experiment,
                                               environment,
                                               simulation.sim_id,
                                               gzserver_host)

        simulation.cle.cle_handler = SimulationCLEHandler(simulation)

        logger.info("simulation initialized")
        if using_collab_storage:
            path_to_cloned_configuration_folder = os.path.split(experiment)[0]
            if tempfile.gettempdir() in path_to_cloned_configuration_folder:
                logger.debug(
                    "removing the temporary configuration folder %s",
                    path_to_cloned_configuration_folder
                )
                shutil.rmtree(path_to_cloned_configuration_folder)

    except IOError as e:
        raise NRPServicesGeneralException(
            "Error while accessing simulation models (" + e.message + ")",
            "Models error")
    except rospy.ROSException as e:
        raise NRPServicesGeneralException(
            "Error while communicating with the CLE (" + e.message + ")",
            "CLE error")


def clean(simulation):
    """
    Cleans a simulation after the simulation has failed

    :param simulation: The simulation
    :param previous_state: The last clean state
    """
    # Make sure we have no zombie processes
    simulation.state_machine_manager.terminate_all()
