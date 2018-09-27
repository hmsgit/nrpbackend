##
## Target All Components in Package by Default (e.g. make verify)
##

#modules that have tests
TEST_MODULES=hbp_nrp_commons/hbp_nrp_commons/ hbp_nrp_watchdog/hbp_nrp_watchdog/ hbp_nrp_cleserver/hbp_nrp_cleserver/ hbp_nrp_backend/hbp_nrp_backend/

#modules that are installable (ie: ones w/ setup.py)
INSTALL_MODULES=hbp-flask-restful-swagger-master hbp_nrp_commons hbp_nrp_watchdog hbp_nrp_cleserver hbp_nrp_backend

#packages to cover
COVER_PACKAGES=hbp_nrp_commons hbp_nrp_watchdog hbp_nrp_cleserver hbp_nrp_backend

#documentation to build
DOC_MODULES=hbp_nrp_commons/doc hbp_nrp_watchdog/doc hbp_nrp_cleserver/doc hbp_nrp_backend/doc

PYTHON_PIP_VERSION?=pip==9.0.3

##
## Individual Component Release Targets
##
verify-hbp_nrp_commons:
	$(MAKE) verify TEST_MODULES=hbp_nrp_commons/hbp_nrp_commons/\
                       INSTALL_MODULES=hbp_nrp_commons\
                       COVER_PACKAGES=hbp_nrp_commons\
                       DOC_MODULES=hbp_nrp_commons/doc/\
                       IGNORE_LINT="$(IGNORE_LINT)|hbp_nrp_cleserver|hbp_nrp_backend|hbp_nrp_watchdog|hbp-flask-restful-swagger-master"

##### DO NOT MODIFY BELOW #####################

ifeq ($(NRP_INSTALL_MODE),user)
        include user_makefile
else
        CI_REPO?=git@bitbucket.org:hbpneurorobotics/admin-scripts.git
        CI_DIR?=$(HBP)/admin-scripts/ContinuousIntegration
        THIS_DIR:=$(PWD)

        FETCH_CI := $(shell \
                if [ ! -d $(CI_DIR) ]; then \
                        cd $(HBP) && git clone $(CI_REPO) > /dev/null && cd $(THIS_DIR);\
                fi;\
                echo $(CI_DIR) )

        include $(FETCH_CI)/python/common_makefile
endif
