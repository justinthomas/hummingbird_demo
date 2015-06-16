#!/bin/bash

ROBOT="QuadrotorGolf"

while true; do
  rosservice call /$ROBOT/estop
done

