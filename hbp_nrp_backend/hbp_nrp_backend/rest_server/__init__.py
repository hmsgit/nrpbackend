"""
This package contains the implementation of the rest server to control experiments
"""

__author__ = 'GeorgHinkel'


from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger


class ErrorForwardingApi(Api):
    """
    An API class that forwards error routing using usual Flask
    """
    def error_router(self, original_handler, e):
        """
        Route the error
        :param original_handler: Flask handler
        :param e: Error
        """
        return original_handler(e)

app = Flask(__name__)
api = swagger.docs(ErrorForwardingApi(app), apiVersion='0.1')

# import rospy

# rospy.init_node("REST_server")

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers
from hbp_nrp_backend.rest_server.__SimulationSetup import SimulationSetup
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl

api.add_resource(SimulationSetup, '/simulation')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>/state')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
