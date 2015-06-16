#!/bin/bash

ROBOT="QuadrotorGolf"

echo "Enable motors..."
rosservice call /$ROBOT/motors true
sleep 1

read -p "Press [Enter] to takeoff"
echo "Takeoff..."
rosservice call /$ROBOT/takeoff
sleep 1

read -p "Press [Enter] to prep traj"
echo "Prepping trajectory..."
rosservice call /$ROBOT/prepTraj
sleep 3

read -p "Press [Enter] to hover"
rosservice call /$ROBOT/hover
sleep 3

read -p "Press [Enter] to estop"
rosservice call /$ROBOT/estop
