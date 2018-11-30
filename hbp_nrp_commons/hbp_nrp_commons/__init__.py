"""
This package contains functionality shared across the NRP backend
"""

__author__ = "Georg Hinkel"

from hbp_nrp_commons.version import VERSION as __version__  # pylint: disable=W0611


# https://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
def enum(*sequential, **named):
    """
    Create an enumeration Class.
    usage:
    MyEnum = enum('VALUE_1','VALUE_2','VALUE_3', VALUE_4=4)

    :param sequential: Values of the enum specified as positional string args
    :param named: Values of the enum specified as keyword args
    :return: A new Enum Type having as values the specified parameters
    """
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
