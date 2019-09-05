#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/room.world" & sleep 6
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " & sleep 6
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " & sleep 6
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
