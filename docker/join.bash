#!/usr/bin/env bash

if [ $# -ne 1 ]
then
  echo "Usage: $0 CONTAINER_NAME"
  exit 1
fi

docker exec -it $1 bash