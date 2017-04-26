from RestrictedPython import compile_restricted
from hbp_nrp_cleserver.server.ROSCLEServer import extract_line_number
from unittest import TestCase
import sys

__author__ = "Georg Hinkel"

class TestErrorLineExtraction(TestCase):

    ret_none = lambda: None

    def test_line_number_of_type_error(self):
        code = "# Here are some comments\n" \
               "def test():\n" \
               "    foo = None\n" \
               "    # the next line will crash\n" \
               "    foo.bar\n"
        line_no = self.__get_line_number_of_error(code)
        self.assertEqual(5, line_no)

    def __get_line_number_of_error(self, code):
        test = TestErrorLineExtraction.ret_none
        restricted = compile_restricted(code, '<string>', 'exec')
        exec restricted
        try:
            return test()
        except Exception:
            tb = sys.exc_info()[2]
            return extract_line_number(tb)