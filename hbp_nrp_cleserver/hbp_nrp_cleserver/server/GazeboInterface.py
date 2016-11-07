"""
Interface for gzserver and gzbridge spawning classes.
"""

__author__ = 'Alessandro Ambrosano'


class IGazeboServerInstance(object):
    """
    Takes care of starting a gzserver process somewhere, connect it to the given roscore.
    Each implementation has to take care of providing the methods start, stop and restart as
    well as a property containing the gzserver master URI.
    """

    def __init__(self):
        self.__gazebo_died_callback = None

    @property
    def gazebo_died_callback(self):
        """
        Gets the callback when gazebo dies
        """
        return self.__gazebo_died_callback

    @gazebo_died_callback.setter
    def gazebo_died_callback(self, callback):
        """
        Sets the callback when gazebo dies

        :param callback: The new callback
        """
        self.__gazebo_died_callback = callback

    def _raise_gazebo_died(self):
        """
        Informs clients that Gazebo has died
        """
        if self.__gazebo_died_callback is not None:
            self.__gazebo_died_callback()

    def start(self, ros_master_uri):   # pragma: no cover
        """
        Starts a gzserver instance connected to the local roscore (provided by
        ros_master_uri)

        :param ros_master_uri The ros master uri where to connect gzserver.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def stop(self):   # pragma: no cover
        """
        Stops the gzserver instance.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def gazebo_master_uri(self):   # pragma: no cover
        """
        Returns a string containing the gazebo master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def restart(self, ros_master_uri):   # pragma: no cover
        """
        Restarts the gzserver instance.

        :param ros_master_uri The ros master uri where to connect gzserver.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IGazeboBridgeInstance(object):   # pragma: no cover
    """
    Takes care of starting a gzserver instance somewhere.
    """

    def start(self):
        """
        Starts the gzbridge instance represented by the object.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def stop(self):
        """
        Stops the gzbridge instance represented by the object.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def restart(self):
        """
        Restarts the gzbridge instance represented by the object.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
