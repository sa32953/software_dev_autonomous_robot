#!/bin/bash -e

if rospack list | grep "ex01_my_first_package"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi

roslaunch ex01_my_first_package ex01_my_first_package.launch&

#wait for node to start
sleep 20
if rosnode list | grep "first_node"; then
  echo "node found"
else
  echo "node not found"
  exit 1
fi
