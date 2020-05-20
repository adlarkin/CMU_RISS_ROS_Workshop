#!/usr/bin/env bash

docker build -t riss_ros_tutorials_melodic:latest --build-arg user_id=$(id -u) .