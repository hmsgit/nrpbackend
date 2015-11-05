"""
Unit test for model CollabContext
"""

__author__ = 'peppicel'

import unittest
from hbp_nrp_backend.rest_server.__CollabContext import CollabContext


class TestCollabContext(unittest.TestCase):

    def test_constructor(self):
        test_context = CollabContext('dummy_context_id', 'dummy_experiment_id')
        self.assertEqual(test_context.context_id, 'dummy_context_id')
        self.assertEqual(test_context.experiment_id, 'dummy_experiment_id')

    def test_repr(self):
        test_context = CollabContext('dummy_context_id', 'dummy_experiment_id')
        self.assertEqual(repr(test_context), '<id dummy_context_id>')