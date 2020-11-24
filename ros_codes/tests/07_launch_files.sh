#!/bin/bash -e

if rospack list | grep "ex07_launch_file";then
        echo "package found"
else
        echo "package not found"
        exit 1
fi

roslaunch goal_publisher goal_publisher.launch&
pid=$!
sleep 5
timeout 5 roslaunch ex07_launch_file goalPrint.launch > output
kill -s TERM $pid
wait $pid

regex="\s*[-+]?([0-9]*\.[0-9]+|[0-9]+)"

if grep -Pzo ".*Point 0: x: $regex, y: $regex, z: $regex\n.*Point 1: x: $regex, y: $regex, z: $regex\n.*Point 2: x: $regex, y: $regex, z: $regex\n" output; then
        rm output
        echo "Found the correct point output!"
else
        rm output
        echo "Correct output was not found!"
        exit 1
fi
