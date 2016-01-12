#!/bin/bash
# Script to locally run the integration test.
# Author: Bernd Eckstein

if [ "$1" == "--help" ]; then
    echo
    echo "This script starts the integration test on a local system"
    echo "Syntax: $0 [--bag|--help]"
    echo "           --bag     Start rosbag replay (if you want to run the test like it is run on jenkins"
    echo "           --help    Show this help" 
    echo
    exit
fi

# Start roscore, gzserver and SimulationFactory in Background
roscore &
rosrun gazebo_ros gzserver --verbose &
sleep 2

# Start Rosbag in 12s
if [ "$1" == "--bag" ]; then
    sleep 12 && rosbag play hbp_nrp_backend/hbp_nrp_backend/tests/integration/husky_camera_topic_recording.bag -r 0.8 &
fi

# Starting Integration test
# The integration test will exit with return code 0 if sucessful or 1 if not
python hbp_nrp_backend/hbp_nrp_backend/tests/integration/integration_test.py
result=$?

# Kill roscore, gzserver and rosbag"
killist="roscore gzserver rosbag"
for k in $killist; do
    PID=`ps aux | grep $k | awk '{print $2}'`
    if [ -n "$PID" ];then kill $PID; fi;
done

# Wait for shutdown
sleep 2
echo "Shutdown complete."

# Show Result of the test
if [ "$result" == "0" ]; then
    echo -e "\e[1;32mIntegration Test successful.\e[0m"
else
    echo -e "\e[1;31mIntegration Test failed.\e[0m"
fi

exit $result
