"""
This resource represents the topics contained in a running simulation
"""
from flask_restful import Resource
import rosgraph.masterapi as master


__author__ = "Georg Hinkel"


# pylint: disable=no-self-use
class SimulationTopics(Resource):
    """
    This resource represents the topics contained in a running simulation
    """

    def get(self):
        """
        Gets a list of topics available in the current simulation
        :return:
        """
        topics = []
        m = master.Master('masterapi')
        filtereddict = {
            k: v for k, v in m.getTopicTypes()
            if not k.startswith('/monitor') and
            not k.startswith('/gazebo') and
            not k.startswith('/ros') and
            not k.startswith('/odom') and
            not k.startswith('/clock')
        }
        for k in filtereddict:
            topic_e = {
                'topic': k,
                'topicType': filtereddict[k]
            }
            topics.append(topic_e)
        return {'topics': topics}, 200
