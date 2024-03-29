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

This workshop consists of four days focusing on getting familiar with the basics of ROS2 and applying it to a few scenarios. The workshop material is cloned from these repositories:
* **Tuesday**: Starting from scratch - setting up the tools, running some simulation tools and peeling back the first layer of the ROS 2 environment.
* **Wednesday**: Mobile Robots - Start by working on a simulated environment of a mobile robot and driving autonomously. Move on to the afternoon with real robots.
* **Thursday**: [Manipulation][04-manipulation] and vision - Running a simulated environment of the ur5e robots and then using opencv to process vision based information.
* **Friday**: scripting tools and rosbags - capture real world information for reviewing and make basic scripts to auto start your solution.

#### Workspace Setup

Across these four days, you will incrementally build on your pass knowledge slowly building elements by yourself. Each day will bring different tasks so if you don't finish one day you can always complete it later. 

<details><summary>Try to figure out how to create a `catkin` workspace yourself, otherwise, click here to for answer!</summary>
  
```sh
mkdir -p ~/workshop_ws/src  # Creates a workspace directory named workshop_ws.
cd ~/workshop_ws/src
```

</details>

#### Git Clone

For each day, clone the repository linked below into the
`src/` directory of the workspace `workshop_ws`. 

<details><summary>Try to figure this out yourself first, otherwise, click here to for answer!</summary>

E.g. for the [sensor-integration][01-sensor-integration] repository, you'd type:

```sh
cd ~/workshop_ws/src
git clone https://github.com/ros-workshop/sensor-integration.git
```
Or if you are using SSH keys:
```
cd ~/workshop_ws/src
git clone git@github.com:ros-workshop/sensor-integration.git
```

</details>

#### Colcon Build

Build and then source the workspace. 

<details><summary>Try to figure this out yourself first, otherwise, click here to for answer!</summary>

+ **Note**: If this command fails, install catkin tools following the instructions [here][catkin-tools].

+ **Tip**: Source any workspaces you want to extend before running `catkin build`.


```sh
cd ~/workshop_ws
catkin build
source devel/setup.bash
```
</details>


### Tuesday: Starting from scratch
**Topics**:
+ Work through the startcreatingrobots by John Vial found [here](https://startcreatingrobots.com/)
+ Explore Nodes and Topics from command line

### Wednesday: Mobile Robot
**Topics**:
+ Creating a URDF
+ Visualising your robot
+ Fixing a broken URDF
+ Adding a sensor to a robot
+ Controlling a simulated robot
+ Detecting an obstacle and stopping the robot
+ Port your solution to Pioneer Robot and get it working


+ Creating a map using a lidar
+ Simultaneous Localisation and Mapping (SLAM)
+ Using nav2 for navigation
+ Finding an object by navigating around a map

### Thursday: Manipulators and Vision
**Topics**:
+ Using a camera to detect Apriltags
+ Using a real camera with ROS
+ Camera calibration
+ Fusing lidar and camera/DNN data for person detection and localisation

### Friday: Scripting and working on your own
**Topics**:
+ Creating a Moveit configuration package
+ Moving Your robot in `rviz`
+ Using the Moveit class in a node
+ Creating a OctoMap using a depth camera

### Friday: [Mobility Plus Manipulation][05-mobility-plus-manipulation]
**Topics**:
+ How to integrate multiple ROS nodes together 

[01-sensor-integration]: https://github.com/ros-workshop/sensor-integration
[02-slam-navigation]: https://github.com/ros-workshop/slam-navigation
[03-perception]: https://github.com/ros-workshop/perception
[04-manipulation]: https://github.com/ros-workshop/manipulation
[05-mobility-plus-manipulation]: https://github.com/ros-workshop/mobility-plus-manipulation

[gh-git-flow]: https://guides.github.com/introduction/flow
[gh-ssh-keys]: https://help.github.com/articles/connecting-to-github-with-ssh/

[ros-cmakelists]: http://wiki.ros.org/catkin/CMakeLists.txt
[ros-custom-msg]: http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages
[ros-tutorials]: http://wiki.ros.org/ROS/Tutorials
