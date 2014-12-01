"""
Runs the rest server
"""

__author__ = 'GeorgHinkel'

# pragma: no cover

from hbp_nrp_backend.rest_server import app
import sys

if __name__ == '__main__':
    port = 5000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print "Could not parse port, using default port 5000"
    app.run(port=port)
