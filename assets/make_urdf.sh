#!/bin/bash

cd urdf/robot/src
xacro simple.urdf.xacro > ../robot_gazebo.urdf
cd -

# TODO: add sed/awk for automatically stripping all gazebo tags
