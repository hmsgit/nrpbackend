"""
This module makes some rest requests multi-threadable.
By default, all rest requests are considered not thread safe and are therefore synchronized.
Requests that are known to be thread-safe, are executed concurrently.
To mark a rest request as thread-safe, decorate the Resource function handling the request
(get, post, delete, put) with the decorator @RestSyncMiddleware.threadsafe
"""
from threading import Lock
import logging

logger = logging.getLogger(__name__)


class RestSyncMiddleware(object):
    """
    Middleware that allows thread-safe requests to be executed concurrently
    """

    def __init__(self, wsgi_app, app):
        self.wsgi_app = wsgi_app
        self.app = app
        self.threadLock = Lock()

    @staticmethod
    def threadsafe(func):
        """
        Decorates functions as thread-safe
        """
        func.is_threadsafe = True
        return func

    def __call__(self, environ, start_response):
        pathinfo = environ.get("PATH_INFO")
        method = environ.get("REQUEST_METHOD")

        viewfunction, _ = self.app.url_map.bind(
            'localhost', default_method=method).match(pathinfo)
        viewclass = self.app.view_functions[viewfunction].view_class
        viewfn = getattr(viewclass, method.lower())

        if not hasattr(viewfn, "is_threadsafe"):
            self.threadLock.acquire()

        try:
            res = self.wsgi_app(environ, start_response)
        # pylint: disable=broad-except
        except Exception, e:
            logger.exception(e)
        finally:
            if not hasattr(viewfn, "is_threadsafe"):
                self.threadLock.release()

        return res
