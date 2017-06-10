#!/bin/bash
echo "terminal3.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
rosrun tr1 tr1_node
