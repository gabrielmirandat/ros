#!/bin/bash
echo "terminal2.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
rosrun tr5_simu tr5_node
