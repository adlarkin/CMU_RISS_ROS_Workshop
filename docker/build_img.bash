#!/usr/bin/env bash

if [ $# -ne 1 ]
then
  echo "Usage: $0 IMAGE_NAME"
  exit 1
fi

docker build -t $1 --build-arg user_id=$(id -u) .