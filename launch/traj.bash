#!/bin/bash

ROBOT="QuadrotorGolf"

echo "Enable motors..."
rosservice call /$ROBOT/motors true
sleep 1

echo "Takeoff..."
rosservice call /$ROBOT/takeoff
sleep 4

echo "Prepping trajectory..."
rosservice call /$ROBOT/prepTraj
sleep 4

read -p "Press [Enter] when ready to start the trajectory"
sleep 2

echo "Attempting to start the trajectory..."
rosservice call /$ROBOT/executeTrajectory

read -p "Press [Enter] to hover"
rosservice call /$ROBOT/hover
