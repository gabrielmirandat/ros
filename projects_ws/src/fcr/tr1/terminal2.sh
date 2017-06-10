#!/bin/bash
echo "terminal2.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
roslaunch tr1 teleop.launch