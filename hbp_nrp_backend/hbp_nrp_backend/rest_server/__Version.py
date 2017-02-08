"""
This module contains the REST implementation
for retrieving the Closed Loop Engine and
the Experiment Designer Back-end versions
"""

__author__ = 'AxelVonArnim, LucGuyot'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server.RestSyncMiddleware import RestSyncMiddleware
from cle_ros_msgs import srv

import rospy
import hbp_nrp_backend

# pylint: disable=R0201


class Version(Resource):
    """
    Implements the REST service providing the user with the versions
    of both Closed Loop Engine and the Experiment Designer back-end.
    """

    def __init__(self):
        Resource.__init__(self)
        self.__get_version_service = rospy.ServiceProxy(
            '/ros_cle_simulation/version',
            srv.GetVersion)
        self.__get_version_service.wait_for_service(timeout=10)

    @swagger.model
    class _Version(object):
        """
        Version object containing CLE and ExDBackend versions
        Only used for swagger documentation
        """

        resource_fields = {
            'hbp_nrp_cle': fields.String(),
            'hbp_nrp_backend': fields.String()
        }
        required = ['hbp_nrp_cle', 'hbp_nrp_backend']

    @swagger.operation(
        notes='Gets the versions of the Closed Loop Engine and \
        the Experiment Designer Back-end.'
        ' Possible values are: x.y.z, x.y.z.devW \
        where x,y,z and W are numbers',
        responseClass=_Version.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Success. The versions were retrieved"
            }
        ]
    )
    @RestSyncMiddleware.threadsafe
    def get(self):
        """
        Gets of the Closed Loop Engine and the Experiment Designer back-end.

        :status 200: Success. The versions were retrieved
        """

        # __get_version_service().version can be different from
        # hbp_nrp_cle.__version__
        return {'hbp_nrp_cle': str(self.__get_version_service().version),
                'hbp_nrp_backend': str(hbp_nrp_backend.__version__)}, 200
