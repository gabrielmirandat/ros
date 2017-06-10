#!/bin/bash
echo "terminal1.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
roslaunch tr1 pioneer3at.gazebo.launch
