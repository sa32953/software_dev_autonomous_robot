#!/bin/bash -e

roslaunch sa_192128_miniprj test_laser_scan.launch&

sleep 3

if rosnode list | grep "test_laser_scan"; then
  echo "Getting Laser Data"
else
  echo "Laser Node not found"
  exit 1
fi
