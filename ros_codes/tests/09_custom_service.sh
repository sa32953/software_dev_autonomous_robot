#!/bin/bash -e
if rospack list | grep "ex09_my_service"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi
