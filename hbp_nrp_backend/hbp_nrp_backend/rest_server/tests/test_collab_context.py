"""
Unit test for model CollabContext
"""

__author__ = 'Daniel Peppicelli'

import unittest

from hbp_nrp_backend.rest_server.__CollabContext import CollabContext


class TestCollabContext(unittest.TestCase):

    def test_constructor(self):
        test_context = CollabContext('dummy_context_id', 'dummy_experiment_id', 'dummy_experiment_folder_uuid')
        self.assertEqual(test_context.context_id, 'dummy_context_id')
        self.assertEqual(test_context.experiment_id, 'dummy_experiment_id')
        self.assertEqual(test_context.experiment_folder_uuid, 'dummy_experiment_folder_uuid')

    def test_repr(self):
        test_context = CollabContext('dummy_context_id', 'dummy_experiment_id', 'dummy_experiment_folder_uuid')
        self.assertEqual(repr(test_context), '<id dummy_context_id>')