"""
Runs the rest server
"""

__author__ = 'GeorgHinkel'

# pragma: no cover

from hbp_nrp_backend.rest_server import app

if __name__ == '__main__':
    app.run(port=5000)
