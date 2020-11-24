#!/bin/bash -e

if rospack list | grep "ex01_my_first_package"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi
