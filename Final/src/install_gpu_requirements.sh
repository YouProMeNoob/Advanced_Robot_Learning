#!/bin/bash

echo "Installing CPU and GPU dependencies"
pip3 install -r /root/catkin_ws/shared/requirements.txt
pip3 install -r /root/catkin_ws/shared/requirements_gpu.txt

echo "Generating URDF"
cd /root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf
xacro open_manipulator_6dof.urdf.xacro -o open_manipulator_6dof.urdf
sed -i 's|package://open_manipulator_6dof_description/meshes/|/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes/|g' open_manipulator_6dof.urdf

echo "Installing Terminator"
apt-get update
apt-get install terminator
