"""
This package contains the implementation of the rest server to control experiments
"""

__author__ = 'GeorgHinkel'


from flask import Flask
app = Flask(__name__)

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__SimulationControl
import hbp_nrp_backend.rest_server.__SimulationSetup
import hbp_nrp_backend.rest_server.__ErrorHandlers
