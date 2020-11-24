#!/bin/bash -e

if rospack list | grep "ex05_vel_subscriber"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi

roslaunch ex05_vel_subscriber ex05_vel_subscriber.launch&

#wait for node to start
sleep 10

if rosnode list | grep "subscriber"; then
  echo "node found"
else
  echo "node not found"
  exit 1
fi
