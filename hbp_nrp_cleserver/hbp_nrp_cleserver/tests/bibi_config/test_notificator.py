"""
Notificator unit test
"""

import unittest
import logging
from mock import MagicMock
from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler


class TestNotificator(unittest.TestCase):

    def test_notificator(self):
        notify = MagicMock()
        Notificator.register_notification_function(notify)
        Notificator.notify('test_message', 'test_update')
        self.assertEqual(notify.call_count, 1)

class TestNotificatorHandler(unittest.TestCase):

    def test_notificator_handler(self):
        messages = []
        def notify_handler(message, update_progress):
            messages.append(message)
        Notificator.register_notification_function(notify_handler)

        logger = logging.getLogger("TestLogger")
        logger.addHandler(NotificatorHandler())
        logger.setLevel(logging.INFO)
        logger.info("Foo")
        logger.info("%s", "Bar")
        logger.debug("Debug")

        self.assertEqual(2, len(messages))
        self.assertEqual("Foo", messages[0])
        self.assertEqual("Bar", messages[1])

if __name__ == '__main__':
    unittest.main()
