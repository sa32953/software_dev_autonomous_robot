#!/bin/bash -e

if rospack list | grep "ex04_publisher"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi

roslaunch ex04_publisher ex04_publisher.launch&

#wait for node to start
sleep 10

if rosnode list | grep "vel_publisher"; then
  echo "node found"
else
  echo "node not found"
  exit 1
fi

if rostopic echo /cmd_vel -n5 | grep "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
"; then
  echo "message correct"
else
  echo "message incorrect"
  exit 1
fi
