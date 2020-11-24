#!/bin/bash -e

if rospack list | grep "ex08_model_deleter"; then
  echo "package found"
else
  echo "package not found"
  exit 1
fi
