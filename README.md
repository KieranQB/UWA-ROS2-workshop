# ROS2-workshop

## Course Prerequisites

To complete this workshop please arrive with:
* A basic understanding of Ubuntu Linux
* A basic understanding of Python and/or C++
* A laptop running either:
  * [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
* Please follow the [installation instructions here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
  * Ask a mentor if you get stuck, and we'll be happy to help
* Access to the Internet (you'll need to download 1-2 GB)
  * Ask a mentor for Wi-Fi access details
* Access to this GitHub repository (consider [adding a SSH key][gh-ssh-keys])
 
**Note**: While any development environment can be used for this course, for beginners we recommend the free [VSCode](https://code.visualstudio.com/) source code editor.

## Contributions

This course material has been adapted from the ROS 1 course and has been developed by volunteers in their spare time. 
As you work through the workshop, please consider contributing any suggestions or edits back -- it's easy!
Please fork the repository you want to update and do a “pull request“ (PR) when you're ready for us to review. For the full process, read up on “Git Flow” [here](https://docs.github.com/en/get-started/using-github/github-flow).
Otherwise let us know of any issues and we will try to get to it when we can.


## Part 1: Workshop Summary

This workshop consists of three days focusing on getting familiar with the basics of ROS2 and applying it to a few scenarios. The workshop material is cloned from these repositories:
* **Monday**: Starting from scratch - setting up the tools, running some simulation tools and peeling back the first layer of the ROS 2 environment.
* **Tuesday**: Mobile Robots - Start by working on a simulated environment of a mobile robot building up the robot and the environment. Test out manipulator robots at the end.
* **Wednesday**: Making your own software stack - Go through the ROS2 environment and files. Setup your project to use SLAM for mapping.
* **Thursday**: scripting tools, rosbags, ros_control and lifecycle - capture real world information for reviewing and make basic scripts to auto start your solution. Introduction to ros_control, a tool for fast low level communications and lifecycle nodes that better manage the flow of the system.


### Monday: Starting from scratch
**Topics**:
+ Work through the startcreatingrobots by John Vial found [here](https://startcreatingrobots.com/)
+ Explore Nodes and Topics from command line
+ Build a ROS2 simulation for the UR5/e robot arms

### Tuesday: Manipulator and Mobile Robots
**Topics**:
+ Build a ROS2 simulation for a pioneer robot
+ Creating a URDF
+ Visualising your robot
+ Fixing a broken URDF
+ Adding a sensor to a robot
+ Controlling a simulated robot
+ Detecting an obstacle and stopping the robot
+ Build up the world environment as you see fit


### Wednesday: Setup your own system
**Topics**:
+ Setup the file structure
+ create a launch file
+ Creating a map using a lidar
+ Simultaneous Localisation and Mapping (SLAM)
+ Using nav2 for navigation
+ Finding an object by navigating around a map
+ create your own node
+ create your own Topics, Services and Actions

+ Using a camera to detect Apriltags
+ Using a real camera with ROS
+ Camera calibration


### Extras: Scripting, ROSbags, ros2_control and lifecycle nodes
**Topics**:
+ Basic scripting ideas common in robotics
+ Use of rosbags to capture data and diagnose issues
+ ros2_control for low level actuation
+ Lifecycle nodes

