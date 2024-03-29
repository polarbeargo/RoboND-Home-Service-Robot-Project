[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

[//]: # (Image References)
[image1]: ./images/2Dnav.gif
[image2]: ./images/homeService.gif
[image3]: ./images/wall.gif
[image4]: ./images/pickObject.gif
[image5]: ./images/addMarkersh.gif

# RoboND-PathPlanning
A wall_follower ROS C++ node for the Home Service Robot Project. This node will autonomously drive your robot close to the walls while avoiding obstacles on its path.

### Basic Idea
A wall follower algorithm is a common algorithm that solves mazes. This algorithm is also known as the left-hand rule algorithm or the right-hand rule algorithm depending on which is your priority. The wall follower can only solve mazes with connected walls, where the robot is guaranteed to reach the exit of the maze after traversing close to walls. You will implement this basic algorithm in your environment to travel close to the walls and autonomously map it.

Here’s the wall follower algorithm(the left-hand one) at a high level:
``` 
If left is free:
    Turn Left
Else if left is occupied and straight is free:
    Go Straight
Else if left and straight are occupied:
    Turn Right 
Else if left/right/straight are occupied or you crashed:
    Turn 180 degrees
```

This algorithm has a lot of disadvantages because of the restricted space it can operate in. In other words, this algorithm will fail in open or infinitely large environments. Usually, the best algorithms for autonomous mapping are the ones that go in pursuit of undiscovered areas or unknown grid cells.

### Task List
Here's a detailed task list of the steps you should take in order to implement this package with your home service robot to autonomously map an environment:
1. Create a **wall_follower** package.
2. Create a **wall_follower** C++ node by cloning this repo.
3. Edit the wall_follower C++ **node name** and change it to **wall_follower**.
4. Edit the wall_follower C++ subscriber and publisher **topics name**.
5. Write a **wall_follower.sh** shell script that launch the **turtlebot_world.launch**, **gmapping_demo.launch**, **view_navigation.launch**, and the **wall_follower** node.
6. Edit the **CMakeLists.txt** file and add directories, executable, and target link libraries.
7. Build your **catkin_ws**.
8. Run your **wall_follower.sh** shell script to autonomously map the environment.
9. Once you are satisfied with the map, kill the wall_follower terminal and save your map in both **pgm** and **yaml** formats in the **World** directory of your **catkin_ws/src**.

# RoboND-Home-Service-Robot

The final project of the Udacity Robotics Software Engineer Nanodegree combines a number of elements from previous projects use everything you learned in the Nanodegree Program to build a Home Service Robot in ROS:
1. Implement multiple shell scripts for convenient test project.
2. Design enviroment using Gazebo with building editor
3. Manually teleoperate designed robot and test SLAM using gmapping and teleop.
4. Apply Adaptive Monte Carlo Localisation to detect the robot position within the known map create a wall_follower node that autonomously drives designed robot to map your environment.  
5. Use the ROS move_base library to plot a path to a target pose and navigate to it.
6. Write a pick_objects node to encompass the path planning and driving libraries, listening for goal poses.
7. Write a add_markers node to publish goal poses for the robot, then compare these to the actual pose (odometry topic) to determine success.


# Installation
This repository is intended to run only on Linux Ubuntu v16.04 with ROS Kinetic. Create a Catkin Workspace as explained [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

To install, clone the repository to /home/workspace. The command below will pull all required submodules and copy directly to Catkin Workspace.
`git clone --recurse-submodules https://github.com/Polarbeargo/RoboND-Home-Service-Robot-Project.git`

Import the following packages and install them in the `src` directory of `catkin workspace`. Be sure to clone the full GitHub directory and not just the package itself.

This repository already includes official ROS packages compatible with this repository: 
* [gmapping](https://github.com/ros-perception/slam_gmapping)
* [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop) 
* [turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions)
* [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator)  

Their dependencies must be installed to succesfully use this repository:  
```
cd /home/workspace/catkin_ws/src
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```
Once all the necessary package are installed, run the following commands from the catkin_ws directory:
```
cd ..
catkin_make
source devel/setup.bash   
sudo chmod 7 src/ShellScripts/*.sh
```

# Running the Simulation
From the `catkin_ws/` directory run the following commands:  

Testing SLAM
``` bash
$ ./src/ShellScripts/test_slam.sh
```  
![][image3]  
Testing Navigation
``` bash
$ ./src/ShellScripts/test_navigation.sh
```  
![][image1]  
Pick_objects
``` bash
$ ./src/ShellScripts/pick_objects.sh
```  
![][image4]  

Add Markers
``` bash
$ ./src/ShellScripts/add_marker.sh
```  
![][image5]
Home Service
```bash
$ ./src/ShellScripts/home_service.sh
```
![][image2]
