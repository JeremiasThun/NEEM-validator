#!/bin/bash

cd /home/ros/src/neem_validator
git pull
source /source_ros.sh
roslaunch /home/ros/src/neem_validator/launch/validator.launch path_to_neem:=/neem_data
