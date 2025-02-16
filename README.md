# ROS2-workshop

## Course Prerequisites

To complete this workshop please arrive with:
* A basic understanding of Ubuntu Linux
* A basic understanding of Python and/or C++
* A laptop running either:
  * [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* Please follow the [installation instructions here](./ros-installation.md)
  * Ask a mentor if you get stuck, and we'll be happy to help
* Access to the Internet (you'll need to download 1-2 GB)
  * Ask a mentor for Wi-Fi access details
* Access to this GitHub repository (consider [adding a SSH key][gh-ssh-keys])
 
**Note**: While any development environment can be used for this course, for beginners we recommend the free [VSCode](https://code.visualstudio.com/) source code editor.

## Contributions

This course material has been adapted from the ROS 1 course and has been developed by volunteers in their spare time. 
As you work through the workshop, please consider contributing any suggestions or edits back -- it's easy!
Please fork the repository you want to update and do a “pull request“ (PR) when you're ready for us to review. For the full process, read up on “Git Flow” [here][gh-git-flow].
Otherwise let us know of any issues and we will try to get to it when we can.


## Part 1: Workshop Summary

This workshop consists of three days focusing on getting familiar with the basics of ROS2 and applying it to a few scenarios. The workshop material is cloned from these repositories:
* **Monday**: Starting from scratch - setting up the tools, running some simulation tools and peeling back the first layer of the ROS 2 environment.
* **Tuesday**: Mobile Robots - Start by working on a simulated environment of a mobile robot and driving autonomously. Move on to the afternoon with real robots.
* **Wednesday**: [Manipulation][04-manipulation] and vision - Running a simulated environment of the ur5e robots and then using opencv to process vision based information.
* **Extra Interests**: scripting tools and rosbags - capture real world information for reviewing and make basic scripts to auto start your solution.


### Monday: Starting from scratch
**Topics**:
+ Work through the startcreatingrobots by John Vial found [here](https://startcreatingrobots.com/)
+ Explore Nodes and Topics from command line
+ Build a ROS2 simulation for the UR5/e robot arms

### Tuesday: Manipulator and Mobile Robots
**Topics**:
+ Build a ROS2 simulation for the UR5/e robot arms
+ Creating a URDF
+ Visualising your robot
+ Fixing a broken URDF
+ Adding a sensor to a robot
+ Controlling a simulated robot
+ Detecting an obstacle and stopping the robot

+ Creating a map using a lidar
+ Simultaneous Localisation and Mapping (SLAM)
+ Using nav2 for navigation
+ Finding an object by navigating around a map

### Wednesday: Real Robots and Vision
**Topics**:
+ Porting simulation environment to real Pioneers
+ Setup UR5e control using moveit
+ Using a camera to detect Apriltags
+ Using a real camera with ROS
+ Camera calibration


### Extras: Scripting and working on your own
**Topics**:
+ Learn about some extra tools that are useful when working with ROS2

