"""
This class tests the common tools of the main module.
"""
__author__ = 'LucGuyot'

from hbp_nrp_backend import get_date_and_time_string
import unittest


class TestUtilities(unittest.TestCase):

    def test_get_date_and_time_string(self):
        # Create a string that can be used to generate a file or a folder name
        # e.g., 2016-03-10_12-11-36, used in csv-recorders-2016-03-10_12-11-36
        time = get_date_and_time_string()
        # Some special characters should not be used in file or folder names
        self.assertNotIn(' ', time)
        self.assertNotIn('/', time)
        self.assertNotIn('\"', time)
        self.assertNotIn('\'', time)
        # The later the file or the folder is created, the lower it appears in a sorted list
        later_time = get_date_and_time_string()
        time_list = [time, later_time]
        time_list.sort()
        self.assertEqual(time_list[0], time)


if __name__ == '__main__':
    unittest.main()
