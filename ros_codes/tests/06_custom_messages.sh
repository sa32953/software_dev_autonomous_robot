#!/bin/bash -e

if rosmsg show Age; then
  echo "Message found"
else
  echo "Message not found"
  exit 1
fi


if rosmsg show Age | grep "float32\|int" ; then
  echo "Message fields ok"
else
  echo "Message fields not ok"
  exit 1
fi
