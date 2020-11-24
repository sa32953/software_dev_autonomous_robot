#!/bin/bash -e

source ~/catkin_ws/devel/setup.bash
roscd ex01_my_first_package

cat src/first_node.py
cd launch
cat ex01_my_first_package.launch
