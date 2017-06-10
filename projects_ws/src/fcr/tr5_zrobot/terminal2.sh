#!/bin/bash
echo "terminal2.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
rosrun tr5_zrobot tr5_zrobot_node
