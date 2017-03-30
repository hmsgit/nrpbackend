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
        m = master.Master('masterapi')
        filtereddict = {
            k: v for k, v in m.getTopicTypes()
            if not k.startswith('/monitor') and
            not k.startswith('/gazebo') and
            not k.startswith('/ros') and
            not k.startswith('/odom') and
            not k.startswith('/clock')
            }
        topics = []
        topic_names = []
        system_state = m.getSystemState()
        # consider subscribers and publishers, but not services
        for topic_list in system_state[0:2]:
            for topic_info in topic_list:
                topic_name = topic_info[0]
                if (
                    topic_name in filtereddict and
                    not topic_name in topic_names
                ):
                    topic_e = {
                        'topic': topic_name,  # e.g., '/husky/cmd'
                        'topicType': filtereddict[topic_name]  # e.g., '/gazebo'
                    }
                    topics.append(topic_e)
                    topic_names.append(topic_name)
        return {'topics': topics}, 200
