#!/usr/bin/env python
"""
This module contains the classes needed to have all gazebo services running on the Lugano viz
cluster.
"""

import re
import pexpect
import time
import logging
import os
import sys
import netifaces
import datetime
from hbp_nrp_cleserver.server.GazeboInterface import IGazeboServerInstance
# We will monitor the remote Gazebo via a remote watchdog, but this implementation is not yet ready
# pylint: disable=unused-import
from hbp_nrp_cleserver.server.WatchdogServer import WatchdogClient

# Info messages sent to the notificator will be forwarded as notifications
notificator = logging.getLogger('hbp_nrp_cle.user_notifications')
logger = notificator.parent


# This will be the method called by the pexpect object to log.
# pexpect is expecting kwargs unused argument.
def _log_write(*args, **kwargs):  # pylint: disable=unused-argument
    """
    Translation between pexpect log to file mechanism and python logging module
    """
    content = args[0]
    # let's ignore other params, pexpect only use one arg AFAIK
    if content in [' ', '', '\n', '\r', '\r\n']:
        return  # don't log empty lines
    for eol in ['\r\n', '\r', '\n']:
        # remove ending EOL, the logger will add it anyway
        content = re.sub(r'\%s$' % eol, '', content)
    return logger.info(content)  # call the logger info method with the reworked content


def _set_up_logger():
    """
    Configure the root logger of the application
    :param: logfile_name: name of the file created to collect logs
    """
    # give the logger the methods required by pexpect
    logger.write = _log_write
    logger.flush = lambda: None

_set_up_logger()


class LuganoVizClusterGazebo(IGazeboServerInstance):
    """
    Represents an instance of gzserver running on the Lugano viz cluster.

    There is a wide usage of pexpect in this class because of the remote connections via
    ssh needed to access the Lugano machines. Every time a remote command is launched,
    expect() is used to match strings with the shell output in order to understand its status.
    """

    CLUSTER_SLURM_FRONTEND = 'ssh -K bbpnrsoa@bbpviz1.cscs.ch'
    # SLURM salloc calls allocates a node on the cluster. From salloc man page:
    #
    # salloc - Obtain a SLURM job allocation (a set of nodes), execute a command,and then release
    # the allocation when the command is finished.
    # SYNOPSIS
    # salloc [options] [<command> [command args]]
    #
    # -c, --cpus-per-task=<ncpus>
    # Advise the SLURM controller that ensuing job steps will require ncpusnumber of processors
    # per task. Without this option, the controller willjust try to allocate one processor per
    # task.
    # For instance,consider an application that has 4 tasks, each requiring 3 processors. If
    # ourcluster is comprised of quad-processors nodes and we simply ask for12 processors, the
    # controller might give us only 3 nodes. However, by usingthe --cpus-per-task=3 options, the
    # controller knows that each task requires3 processors on the same node, and the controller
    # will grant an allocationof 4 nodes, one for each of the 4 tasks.
    #
    # -I, --immediate[=<seconds>]
    # exit if resources are not available within thetime period specified.If no argument is given,
    # resources must be available immediatelyfor the request to succeed.By default, --immediate is
    # off, and the commandwill block until resources become available. Since this option'sargument
    # is optional, for proper parsing the single letter option mustbe followed immediately with the
    # value and not include a space betweenthem. For example "-I60" and not "-I 60".
    #
    # --gres=<list>
    # Specifies a comma delimited list of generic consumable resources.The format of each entry on
    # the list is "name[:count[*cpu]]".The name is that of the consumable resource.The count is the
    # number of those resources with a default value of 1.The specified resources will be allocated
    # to the job on each nodeallocated unless "*cpu" is appended, in which case the resourceswill
    # be allocated on a per cpu basis.The available generic consumable resources is configurable
    # by the systemadministrator.A list of available generic consumable resources will be printed
    # and thecommand will exit if the option argument is "help".Examples of use
    # include "--gres=gpus:2*cpu,disk=40G" and "--gres=help".
    #
    # -t, --time=<time>
    # Set a limit on the total run time of the job allocation. If therequested time limit exceeds
    # the partition's time limit, the job willbe left in a PENDING state (possibly indefinitely).
    # The default timelimit is the partition's default time limit. When the time limit is reached,
    # each task in each job step is sent SIGTERM followed by SIGKILL. Theinterval between signals
    # is specified by the SLURM configurationparameter KillWait. A time limit of zero requests
    # that no timelimit be imposed. Acceptable time formats include "minutes","minutes:seconds",
    # "hours:minutes:seconds", "days-hours","days-hours:minutes" and "days-hours:minutes:seconds".
    #
    # -p, --partition=<partition_names>
    # Request a specific partition for the resource allocation. If not specified,the default
    # behavior is to allow the slurm controller to select the defaultpartition as designated by
    # the system administrator. If the job can use morethan one partition, specify their names
    # in a comma separate list and the oneoffering earliest initiation will be used.
    #
    # -A, --account=<account>
    # Charge resources used by this job to specified account.The account is an arbitrary string.
    # The account name maybe changed after job submission using the scontrolcommand.
    #
    # --reservation=<name>
    # Allocate resources for the job from the named reservation.

    ALLOCATION_TIME = datetime.timedelta(hours=10)
    DEALLOCATION_COMMAND = 'scancel %s'
    CURRENT_NODES_COMMAND = 'squeue -u bbpnrsoa -t PENDING,RUNNING -h -o "%N"'
    NODE_DOMAIN = '.cscs.ch'
    # Timeout used for pexpect ssh connection calls.
    TIMEOUT = 20
    # Timeout used for pexpect calls that should return immediately (default pexpect timeout is 30
    # seconds).
    SMALL_TIMEOUT = 2
    DEFAULT_GZSERVER_PORT = 11345

    def __init__(self, timezone=None, reservation=None):
        super(LuganoVizClusterGazebo, self).__init__()

        self.__allocation_process = None
        self.__x_server_process = None
        self.__remote_xvnc_process = None
        self.__gazebo_remote_process = None
        self.__remote_display_port = -1
        self.__job_ID = None
        # Holds the state of the SLURM job. The states are defined in SLURM.
        self.__state = "UNDEFINED"
        self.__node = None

        self.__allocation_time = None
        if timezone is not None:
            self.__allocation_time = datetime.datetime.now(
                timezone) + LuganoVizClusterGazebo.ALLOCATION_TIME

        if (reservation is not None):
            reservation = str(reservation)
        else:
            reservation = ''
        LuganoVizClusterGazebo.ALLOCATION_COMMAND = (
            "salloc --immediate=25" +
            " --time=" + str(LuganoVizClusterGazebo.ALLOCATION_TIME) +
            " --reservation=" + reservation +
            " -p interactive -c 4 --account=proj30 --gres=gpu:1"
        )

    def __spawn_ssh_SLURM_frontend(self):
        """
        Return a pexpect object connected to the SLURM frontend.
        SLURM (Simple Linux Utility for Resource Management) is the entry point to
        allocate or manage jobs on the cluster.
        """
        ssh_SLURM_frontend_process = pexpect.spawn('bash',
                                                   logfile=logger)
        ssh_SLURM_frontend_process.sendline(self.CLUSTER_SLURM_FRONTEND)
        result = ssh_SLURM_frontend_process.expect(['[bbpnrsoa@bbpviz1 ~]$',
                                                    'password',
                                                    pexpect.TIMEOUT], self.TIMEOUT)
        if result == 1:
            raise(Exception("SLURM front-end node can't be used without password."))
        if result == 2:
            raise(Exception("Cannot connect to the SLURM front-end node."))

        return ssh_SLURM_frontend_process

    def __allocate_job(self):
        """
        Allocate a new job on the cluster. Return a pexpect object with a ssh
        connection to the allocated node. If once call exit on this object, the
        allocation will be cancelled.
        """
        notificator.info("Requesting resources on the cluster")
        self.__allocation_process = self.__spawn_ssh_SLURM_frontend()

        # clear the buffer before issuing the commands, otherwise parsing the final
        # list may fail with ssh banner header info filling the buffer
        result = self.__allocation_process.expect([pexpect.TIMEOUT,
                                                   '[bbpnrsoa@bbpviz1 ~]$'], self.TIMEOUT)
        if result == 0:
            raise(Exception("Cannot connect to the SLURM front-end node."))

        # determine which nodes are already in use, we can't currently support
        # running multiple gzserver instances on the same backend (yet)
        self.__allocation_process.sendline(self.CURRENT_NODES_COMMAND)
        result = self.__allocation_process.expect([pexpect.TIMEOUT,
                                                   '[bbpnrsoa@bbpviz1 ~]$'], self.TIMEOUT)
        if result == 0:
            raise(Exception("Cannot retrieve list of currently allocated SLURM nodes."))

        # get the list of current nodes between our command and the new prompt
        used_nodes = self.__allocation_process.before.replace('\r', '')  # replace all ^M
        used_nodes = used_nodes.rsplit(self.CURRENT_NODES_COMMAND, 1)[1]
        used_nodes = used_nodes.split('[bbpnrsoa@bbpviz1 ~]$', 1)[0].strip()
        used_nodes = used_nodes.replace('\n', ',')

        # exclude any currently used nodes, otherwise allow any
        if used_nodes:
            logger.info('Currently allocated bbpnrsoa nodes: %s' % used_nodes)
            self.__allocation_process.sendline(self.ALLOCATION_COMMAND + (' -x %s' % used_nodes))
        else:
            self.__allocation_process.sendline(self.ALLOCATION_COMMAND)
        result = self.__allocation_process.expect(['Granted job allocation ([0-9]+)',
                                                   'Submitted batch job [0-9]+',
                                                   'error: spank-auks: cred forwarding failed',
                                                   'error: Unable to allocate resources',
                                                   'error: .+'])
        if result == 2:
            raise(Exception("Kerberos authentication missing"))
        elif result == 3:
            raise(Exception("No resources available on the cluster. "
                            "Try again later."))
        elif result == 4:
            raise(Exception("Job allocation failed: " + str(self.__allocation_process.after)))

        # Find out which node has been allocated
        self.__job_ID = self.__allocation_process.match.groups()[0]
        self.__allocation_process.sendline("scontrol show job " + str(self.__job_ID))
        self.__allocation_process.expect('JobState=[A-Z]+')
        self.__state = self.__allocation_process.after[9:]
        if self.__state != 'RUNNING':
            raise Exception("Job is not running.")
        self.__allocation_process.expect(r' NodeList=(\w+)')
        self.__node = self.__allocation_process.match.groups()[0]

    def __deallocate_job(self):
        """
        Deallocate a job previously allocated through __allocate_job. The idea is
        that exiting from the terminal opened through the allocation causes SLURM
        to complete the Job.
        """
        if self.__allocation_process is not None:
            self.__allocation_process.sendline('exit')
            self.__allocation_process.terminate()
            self.__allocation_process = None
            self.__job_ID = None
            self.__state = "UNDEFINED"
            self.__node = None

    def __start_fake_X(self):
        """
        Start an in memory graphical server. Xvfb or X virtual framebuffer is a display server
        implementing the X11 display server protocol. In contrast to other display servers Xvfb
        performs all graphical operations in memory without showing any screen output. The goal
        is to be able to use vglconnect from the local node to the remote viz cluster node. For
        that, we do need an XServer.

        Xvfb startup should generally produce no output, but handle the following cases:
        - server is already running (e.g. started by root and could not be killed, valid)
        - server needs to initialize GPU/hardware extensions (valid but strange configuration)
        - no output (expected, handled by short timeout with no output)
        - failure to launch (EOF when process reports failure, invalid so abort)
        """
        notificator.info('Starting backend graphics server')
        self.__x_server_process = pexpect.spawn('Xvfb :1', logfile=logger)
        result = self.__x_server_process.expect(
            [
                'Server is already active for display',
                'Initializing built-in extension',
                pexpect.TIMEOUT,    # no output (expected)
                pexpect.EOF
            ],       # crash/failed launch
            self.SMALL_TIMEOUT
        )

        if result == 3:
            raise(XvfbXvnError("Cannot start Xvfb"))

    def __spawn_vglconnect(self):
        """
        Return a pexpect object connected to the allocated viz cluster node.
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))

        vglconnect_process = pexpect.spawn('bash',
                                           env={"DISPLAY": ":1",
                                                "HOME": "/home/bbpnrsoa"},
                                           logfile=logger)
        vglconnect_process.sendline(('vglconnect bbpnrsoa@' +
                                     self.__node +
                                     self.NODE_DOMAIN))
        # We do expect a prompt here
        result = vglconnect_process.expect([r'\[bbpnrsoa@' + self.__node + r'\ ~\]\$',
                                            'password',
                                            pexpect.TIMEOUT])
        if result == 1:
            raise(Exception("Viz cluster node can't be used without password."))
        if result == 2:
            raise(Exception("Cannot connect to node."))

        # Always load virtualgl on vgl connections to be able to use a 3D display
        vglconnect_process.sendline('module load virtualgl')
        vglconnect_process.sendline('export VGL_FORCEALPHA=1')  # force 32-bit buffers

        return vglconnect_process  # This object has to live until the end.

    def __clean_remote_files(self):
        """
        Remove the temporary remote working files
        """
        if (self.__node is not None and self.__allocation_process is not None):
            clean_process = self.__spawn_vglconnect()
            # Clean all old Xvnc lock files.
            clean_process.sendline('rm -rf /tmp/.X*')
            clean_process.terminate()

    def __start_xvnc(self):
        """
        Start a remote Xvnc server. This is the only (known to us) way to have Gazebo using
        the graphic card.
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))

        notificator.info('Starting cluster node graphics server')
        self.__remote_xvnc_process = self.__spawn_vglconnect()

        # Cleanup any leftover Xvnc sessions from our user on this cluster node from failed
        # sessions (e.g. crashes or other rare network issues). This won't impact other cluster
        # node users or their running Xvnc sessions.
        self.__remote_xvnc_process.sendline('killall -9 Xvnc')

        # Find the first available Xvnc port to use, we are not the only cluster user so we cannot
        # guarantee that no other Xvnc is running on a port or that we have access to running
        # sessions. Ohter users may also spawn instances arbitrarily, this ensures a valid session.
        for p in xrange(10, 100):
            self.__remote_xvnc_process.sendline('Xvnc :' + str(p))
            result = self.__remote_xvnc_process.expect([
                'created VNC server for screen 0',
                'Server is already active for display',
                'server already running',
                pexpect.TIMEOUT], self.TIMEOUT
            )

            # valid Xvnc session spawned on port, stop searching
            if result == 0:
                self.__remote_display_port = p
                return

            # timeout while trying to start Xvnc, abort
            elif result == 3:
                raise(XvfbXvnError("Cannot start Xvnc, unknown error."))

        # unable to find an open Xvnc port (very unlikely), abort
        raise(XvfbXvnError("Cannot start Xvnc, no open display ports."))

    def __start_gazebo(self, ros_master_uri):
        """
        Start gazebo on the remote server
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))
        if self.__remote_display_port == -1:
            raise(Exception("Gazebo needs a remote X Server running"))

        notificator.info('Configuring the cluster node environment')
        self.__gazebo_remote_process = self.__spawn_vglconnect()

        # Kill any active gzservers (this should never happen).
        self.__gazebo_remote_process.sendline('killall -9 gzserver')

        # source environment modules init file
        self.__gazebo_remote_process.sendline('source /opt/rh/python27/enable')
        self.__gazebo_remote_process.sendline('source /usr/share/Modules/init/bash 2> /dev/null')

        # determine deployment environment based on an environment variable defined by Puppet
        # (either dev or staging)
        environment = os.environ.get('ENVIRONMENT')

        # set the ENVIRONMENT variable on the cluster side in order
        # to load the appropriate modules there.
        self.__gazebo_remote_process.sendline('export ENVIRONMENT=' + environment)

        # access the dev or staging project resources
        proj_path = '/gpfs/bbp.cscs.ch/project/proj30/neurorobotics/%s/' % environment

        # load modules using the versions specified in the nrp_variables script generated by Puppet
        # (versions are environment dependent)
        if environment == 'staging':
            with open(os.environ.get('NRP_VARIABLES_PATH')) as f:
                content = f.readlines()
            versions = [x.strip() for x in content if '_VERSION=' in x]
            for v in versions:
                self.__gazebo_remote_process.sendline(v)

        # loading the environment modules configuration files
        modules_path = proj_path + 'server-scripts/nrp-services-modules.sh'
        self.__gazebo_remote_process.sendline('source %s' % modules_path)
        result = self.__gazebo_remote_process.expect(['NRP modules loaded.',
                                                      pexpect.TIMEOUT])

        if result == 1:
            raise(Exception("Error while configuring cluster node, gpfs may not be mounted." +
                            str(self.__gazebo_remote_process.after)))

        # activate ROS python venv
        self.__gazebo_remote_process.sendline('source $ROS_PYTHON_VENV/bin/activate')

        # configure variables after all module loads (that could overwrite values)
        self.__gazebo_remote_process.sendline('export DISPLAY=:' + str(self.__remote_display_port))
        self.__gazebo_remote_process.sendline('export ROS_MASTER_URI=' + ros_master_uri)

        # Use the appropriate dev or staging models based on this backend version
        self.__gazebo_remote_process.sendline('export GAZEBO_MODEL_PATH=%s/models' % proj_path)

        # disable online (unreachable) model searching, only use local NRP models
        self.__gazebo_remote_process.sendline('export GAZEBO_MODEL_DATABASE_URI=')

        # launch Gazebo with virtualgl, use -nodl to redirect native opengl calls to virtualgl
        notificator.info('Starting Gazebo server on the cluster node')
        self.__gazebo_remote_process.sendline(
            'vglrun -nodl $GAZEBO_BIN_DIR/gzserver ' +
            '-s $ROS_HBP_PACKAGES_LIB_DIR/libgazebo_ros_api_plugin.so ' +
            '-s $ROS_HBP_PACKAGES_LIB_DIR/libgazebo_ros_paths_plugin.so ' +
            '--verbose')

        result = self.__gazebo_remote_process.expect(['Gazebo multi-robot simulator',
                                                      pexpect.TIMEOUT])

        if result == 1:
            raise(Exception("Error while starting gazebo: " +
                            str(self.__gazebo_remote_process.after)))

    def start(self, ros_master_uri):
        """
        Start gzserver on the Lugano viz cluster
        """
        try:
            self.__allocate_job()
            self.__start_fake_X()
            self.__start_xvnc()
            self.__start_gazebo(ros_master_uri)
        # pylint: disable=broad-except
        except Exception:
            logger.exception('Failure launching gzserver on remote node.')

            # always cleanup, but only raise the start exception up, not any shutdown errors
            try:
                self.stop()
            # pylint: disable=broad-except
            except Exception:
                pass

            raise

    def try_extend(self, new_timeout):
        """"
        Verifies that the gazebo can accept the new simulation timeout
        Returns whether the timeout is accepted
        """
        return self.__allocation_time is None or new_timeout <= self.__allocation_time

    @property
    def gazebo_master_uri(self):
        """
        Returns a string containing the gzserver master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """
        if self.__node is not None:
            return ('http://' +
                    self.__node +
                    self.NODE_DOMAIN +
                    ':' +
                    str(self.DEFAULT_GZSERVER_PORT))
        else:
            return None

    def stop(self):
        # cluster node cleanup (this can fail, but make sure we always release the job below)
        try:
            # terminate running gzserver and invoking bash shell
            if self.__gazebo_remote_process:
                notificator.info('Stopping Gazebo server on the cluster node')
                self.__gazebo_remote_process.sendcontrol('z')
                self.__gazebo_remote_process.sendline('killall -v -9 gzserver')
                self.__gazebo_remote_process.expect([pexpect.TIMEOUT,
                                                     'Killed',
                                                     'gzserver: no process killed'], self.TIMEOUT)
                self.__gazebo_remote_process.terminate()

            # directly terminate Xvnc process (not invoked via bash)
            if self.__remote_xvnc_process:
                notificator.info('Stopping cluster node graphics server')
                self.__remote_xvnc_process.terminate()

            # delete remote locks and any files
            notificator.info('Cleaning up temporary files on the cluster node')
            self.__clean_remote_files()

        # pylint: disable=broad-except
        except Exception:
            logger.exception('Error cleaning up cluster node.')
        finally:
            self.__gazebo_remote_process = None
            self.__remote_xvnc_process = None
            self.__remote_display_port = -1

        # SLURM cleanup (not on the cluster node), always try even if cluster cleanup fails
        if self.__allocation_process:
            notificator.info('Releasing resources on the cluster')
            self.__deallocate_job()

        # cleserver cleanup Xvfb, this is not critical
        if self.__x_server_process:
            notificator.info('Stopping backend graphics server')
            self.__x_server_process.terminate()
            self.__x_server_process = None

    def restart(self, ros_master_uri):
        notificator.info("Restarting Gazebo server on the cluster")
        self.stop()
        self.start(ros_master_uri)


class XvfbXvnError(Exception):
    """
    This exception class is a marker for errors coming from Xvfb or Xvn.
    """
    pass


def _get_roscore_master_uri():
    """
    Return roscore master URI. If the env variable ROS_MASTER_URI is not set,
    then construct it like this: http:// + local_ip + :11311
    """

    master_uri = os.environ.get("ROS_MASTER_URI")
    if not master_uri:
        local_ip = netifaces.ifaddresses('eth0')[netifaces.AF_INET][0]['addr']
        master_uri = 'http://' + local_ip + ':11311'
    return master_uri

# Useful to test out the code.
if __name__ == '__main__':  # pragma: no cover
    logger.setLevel(logging.DEBUG)
    log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logger.setLevel(logging.DEBUG)
    logger.addHandler(console_handler)
    gazebo = LuganoVizClusterGazebo()
    gazebo.start(_get_roscore_master_uri())
    time.sleep(100)
    gazebo.stop()
