This repository contains three packages

1. hbp_nrp_backend
==================
This package is the implementation of the Experiment backend. This includes both the REST server implementation and supportive tools.

2. hbp_nrp_commons
==================
This package contains helper functions; mainly the generated files from pyxb to parse the experiment configuration and bibi files.

3. hbp_nrp_cleserver
====================
This package contains the implementation of CLE Factory and CLE Server


Hint
====
When version bumping, make sure, you bump the version of all packages
(hbp_nrp_backend, hbp_nrp_commons ,hbp_nrp_cleserver)
simutanously (preferably to the same version)
and update the requirements.txt accordingly to the newest version
