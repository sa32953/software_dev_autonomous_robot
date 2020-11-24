#!/bin/bash -e

roslaunch sa_192128_miniprj test_curr_posn.launch&

sleep 3

if rosnode list | grep "test_curr_posn"; then
  echo "Getting Current Posn"
else
  echo "Node not found"
  exit 1
fi
