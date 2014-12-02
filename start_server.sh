#!/bin/bash

cd hbp_nrp_backend

uwsgi --socket 127.0.0.1 --wsgi-file hbp_nrp_backend/runserver.py --callable app &
service nginx restart

cd ..
