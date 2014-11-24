#!/bin/bash

cd hbp_nrp_backend
cd hbp_nrp_backend

uwsgi --socket 127.0.0.1 -w runserver &
service nginx restart

cd ..
cd ..
