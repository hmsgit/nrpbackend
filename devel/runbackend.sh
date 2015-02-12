#!/bin/bash


options=(  "start backend (1) " "start roscore (2) " "start gazebo (3) " "start gazebo client (4) " "Simulation control commands (5) " "press Enter to exit")
commands=("load-experiment (10) " "check-sim (11) " "initialize-sim (12) " "start-sim (13) " "pause-sim (14) " "pause-reset (15) " "pause-stopped (16) " "set-right-screen-red (20) " "set-right-screen-blue (21) " "set-left-screen-red (22) " "set-left-screen-blue (23) " "bright-light-right (30) " "dark-light-right (31) " "bright-light-left (32) " "dark-light-left (33) " "white-light-left (34) " "red-light-left (35) " "green-light-left (36) " "blue-light-left (37) " "white-light-right (38) " "red-light-right (39) " "green-light-right (40) " "blue-light-right (41) " "press Enter to exit" ) 

printf "%s\n" "${options[@]}"

SELECTION=42
while [ $SELECTION -ne 00 ]; do
echo
echo
printf "%s\n" "${options[@]}"
echo
read  -p "Select (1 digit) :" -n 1 SELECTION
case "$SELECTION" in
1) roscore ;;
2) rosrun gazebo_ros gzserver ~/.gazebo/models/virtual_room/virtual_room.sdf ;;
3) rosrun gazebo_ros gzclient;;
4) python $EXDB/hbp_nrp_backend/hbp_nrp_backend/runserver.py ;;
5) python hbp_nrp_cle/hbp_nrp_cle/cle/ROSCLESimulationFactory.py ;;
6) rostopic echo /monitor/population_rate;;
7) echo $SELECTION
SUBSELECTION=42
while [ $SUBSELECTION -ne 00 ]; do
echo
echo
printf "%s\n" "${commands[@]}"
echo
read  -p "Select (2 digits) :" -n 2 SUBSELECTION
echo
case "$SUBSELECTION" in
10) curl -H "Content-Type: application/json" -X POST http://127.0.0.1:5000/simulation -d '{"experimentID": "ExDConf/ExDXMLExample.xml"}' ;;
11) curl -H "Content-Type: application/json" -X GET http://127.0.0.1:5000/simulation/0/state ;;
12) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/state -d '{"state":"initialized"}' ;;
13) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/state -d '{"state": "started"}' ;;
14) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/state -d '{"state": "paused"}' ;;
15) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/state -d '{"state": "reset"}' ;;
16) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/state -d '{"state": "stopped"}' ;;
20) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction -d '{"name": "RightScreenToRed"}' ;;
21) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction -d '{"name": "RightScreenToBlue"}' ;;
22) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction -d '{"name": "LeftScreenToRed"}' ;;
23) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction -d '{"name": "LeftScreenToBlue"}' ;;
30) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "attenuation_linear": 0.01}' ;;
31) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "attenuation_linear": 0.5}' ;;
32) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "attenuation_linear": 0.01}' ;;
33) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "attenuation_linear": 0.5}' ;;
34) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "diffuse": {"r": 255, "g": 255, "b": 255, "a": 255}}' ;;
35) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "diffuse": {"r": 255, "g": 50, "b": 50, "a":255}}' ;;
36) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "diffuse": {"r": 50, "g": 255, "b": 50, "a":255}}' ;;
37) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "right_spot", "diffuse": {"r": 50, "g": 50, "b": 255, "a":255}}' ;;
38) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "diffuse": {"r": 255, "g": 255, "b": 255, "a": 255}}' ;;
39) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "diffuse": {"r": 255, "g": 50, "b": 50, "a":255}}' ;;
40) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "diffuse": {"r": 50, "g": 255, "b": 50, "a":255}}' ;;
41) curl -H "Content-Type: application/json" -X PUT http://127.0.0.1:5000/simulation/0/interaction/light -d '{"name": "left_spot", "diffuse": {"r": 50, "g": 50, "b": 255, "a":255}}' ;;
*) SUBSELECTION=00
esac
done ;;
*) SELECTION=00
esac
done
