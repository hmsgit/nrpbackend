"""
Experiment launcher
This file is generated.
"""
# pragma: no cover

import imp
import multiprocessing
from hbp_nrp_cle.cle.ROSCLEWrapper import ROSCLEClient

__author__ = 'ExD Configuration Script'

cle = None

def initialize():
    """
    Initialize the experiment.
    """
    bibi_script = imp.load_source('bibi_configuration', "{{bibi_script}}")

    #bibi_script.spawn_gazebo_sdf('environment', '{{model}}')

    p = multiprocessing.Process(target=bibi_script.cle_function)
    p.start()

    global cle
    cle = ROSCLEClient()
