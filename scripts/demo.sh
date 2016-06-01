#!/bin/bash

echo "Loading Trajectory"
rosservice call /multi_mav_services/loadTraj

read -p "Press [Enter] to turn on motors"
rosservice call /multi_mav_services/motors true
sleep 1

read -p "Press [Enter] to takeoff"
rosservice call /multi_mav_services/takeoff
sleep 1

read -p "Press [Enter] to prep trajectory"
rosservice call /multi_mav_services/prepTraj "goal: [0.5, 0.0, 0.0, 0.0]"
sleep 1

read -p "Press [Enter] to start trajectory"
rosservice call /multi_mav_services/executeTraj
sleep 1

read -p "Press [Enter] at any time to ehover"
rosservice call /multi_mav_services/ehover
sleep 1

read -p "Press [Enter] to goHome"
rosservice call /multi_mav_services/goHome
sleep 1

read -p "Press [Enter] to land"
rosservice call /multi_mav_services/land
sleep 1

read -p "Press [Enter] to turn off motors"
rosservice call /multi_mav_services/motors false
sleep 1
