# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END

"""
This package provides easy mocking utility for the unit tests
"""

from mock import patch, MagicMock

__author__ = 'Hossain Mahmud'


class MockUtil(object):  # pragma: no cover
    """
    Defines utility functions to create mocks for any given target module.
    Following is an example usage of this class.

    /* mybase.py */
    class MyBase(object):
        def __init__(self):
            self.someAttribute = 'value'

        def my_func(self):
            return 2+2


    /* myderived.py */
    import os
    from some.other.module import AnotherModule
    from some.other.module import DontCareMockingModule
    from mybase import MyBase  # NOTE: it won't work if MyBase is mocked like AnotherModule

    class MyDerived(MyBase):
        def __init__(self):
            self.derivedAttribute = 'value'

        def my_derived_func(self):
            return self.my_func()


    /* test_myderived.py */
    import unittest
    from mock import MagicMock, Mock, patch, ANY
    from hbp_nrp_commons.MockUtil import MockUtil

    from myderived import MyDerived  # module to be tested

    _base_path_ = 'myderived.'  # import path for MyDerived module (myderived.MyDerived)

    @patch(_base_path_ + 'DontCareMockingModule', new=MagicMock())
    class TestMyDerived(unittest.TestCase):
        def setUp(self):
            # mock imported modules
            self.m_os = MockUtil.fakeit(self, _base_path_ + 'os')
            self.m_anotherModule = MockUtil.fakeit(self, _base_path_ + 'AnotherModule')

            # mock the bases
            self.m_baseClass = MockUtil.fake_base(self, MyDerived)  # NOTICE ACTUAL MODULE PASSED

            # now create the object to test
            self.my_derived = MyDerived()  # if needed instantiate in each test individually

        def tearDown(self):
            pass

        def test_derived_func(self):
            # set returns for mocks
            self.m_baseClass.my_func.return_value = 'myval'
            # test for execution
            self.assertEqual(self.my_derived.my_derived_func(), 'myval')
    """

    @classmethod
    def fakeit(cls, test_object, target_module):
        """
        Creates a mock of target_module for test_object class by invoking test_object.patch()
        Automatically adds the newly created patch to be cleaned up on test exit.

        :param test_object: unittest.TestCase class for which target would be mocked,
             Generally, this would be the 'self' of the class running the test cases.
        :param target_module: module to be patched by mock.patch()
        :return: MagicMock() object obtained from patch.start()
        """

        test_object.patch = patch(target_module)
        test_object.addCleanup(test_object.patch.stop)
        return test_object.patch.start()

    @classmethod
    def imitate(cls, *others):
        """
        Creates a MockUtil object that imitates a list of other classes.
        Attributes of the given classes are replaced with MagicMocks.

        :param others: list of classes to be imitated
        :return: MockUtil object containing fake (MagicMock) attributes of all the given classes
        """

        # Set up the attributes
        for other in others:
            for name in other.__dict__:
                try:
                    setattr(cls, name, MagicMock())
                except (TypeError, AttributeError):
                    pass
        return cls

    @staticmethod
    def __reassign_base(target_class, bases):
        """
        Helper function used for restoring base class(es) during the clean-up step
        of a class whose bases have been mocked using MockUtil.fake_base

        WARNING: Underlying logic is bit perplexing. DO NOT CHANGE WITHOUT TESTING the clean-up
        process. Wrong handling might cause base module to remain mocked where actual classes are
        expected.

        :param target_class: class whose bases were mocked and requires to be restored
        :param bases: original bases
        """

        target_class.__bases__ = bases

    @classmethod
    def fake_base(cls, test_object, target_class):
        """

        :param test_object: unittest.TestCase class for which target would be mocked,
             Generally, this would be the 'self' of the class running the test cases.
        :param target_class: class whose bases are to be mocked

        :return: mock class that replaced the bases
        """

        # setup restoration
        test_object.addCleanup(MockUtil.__reassign_base, target_class, target_class.__bases__)
        # mock the base class(es)
        mock = MockUtil.imitate(*target_class.__bases__)
        target_class.__bases__ = (mock, )

        return mock

    @classmethod
    def any(cls):
        """
        Helper function to assert equal to anything. Similar to mock.ANY.
        """
        class Ignore(object):
            """Equal to everything"""
            def __eq__(self, other):
                return True

        return Ignore()
