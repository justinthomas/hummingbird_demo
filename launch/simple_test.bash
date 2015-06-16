#!/bin/bash

ROBOT="QuadrotorGolf"

echo "Enable motors..."
rosservice call /$ROBOT/motors true
sleep 1

read -p "Press [Enter] to takeoff"
rosservice call /$ROBOT/takeoff
sleep 1

read -p "Press [Enter] to land"
rosservice call /$ROBOT/goHome
sleep 1

read -p "Press [Enter] to estop"
rosservice call /$ROBOT/estop
