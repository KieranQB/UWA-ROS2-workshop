# ROS2-workshop

## Course Prerequisites

To complete this workshop please arrive with:
* A basic understanding of Ubuntu Linux
* A basic understanding of Python and/or C++
* A laptop running either:
  * [Ubuntu 20.04](https://releases.ubuntu.com/20.04)
* Please follow the [installation instructions here](./ros-installation.md)
  * Ask a mentor if you get stuck, and we'll be happy to help
* Access to the Internet (you'll need to download 1-2 GB)
  * Ask a mentor for Wi-Fi access details
* Access to this GitHub repository (consider [adding a SSH key][gh-ssh-keys])
 
**Note**: While any development environment can be used for this course, for beginners we recommend the free [VSCode](https://code.visualstudio.com/) source code editor.

## Contributions

This course material has been developed in our spare time. 
As you work through the workshop, please consider contributing any suggestions or edits back -- it's easy! 
Please fork the repository you want to update and do a “pull request“ (PR) when you're ready for us to review. For the full process, read up on “Git Flow” [here][gh-git-flow].



## Part 1: ROS Application Areas

The second part of this workshop consists of four application areas that are worked on over four days. The workshop material is cloned from these repositories:
* **Tuesday**: Starting from scratch
* **Wednesday**: Mobile Robots
* **Thursday**: [Manipulation][04-manipulation]

The fifth day (Friday) integrates these application areas to build a simulated robot that can navigate its environment, finding and picking up cubes:
* **Friday**: [Mobility Plus Manipulation][05-mobility-plus-manipulation]

#### Workspace Setup

Across these five days, you will incrementally build a `catkin` workspace called ```workshop_ws```. Each day will build on the previous, so make sure you end each day with a working solution! 

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
+ Work through the startcreatingrobots by John Vial
+ Explore Nodes and Topics from command line


+ Creating a URDF
+ Visualising your robot
+ Fixing a broken URDF
+ Adding a sensor to a robot
+ Controlling a simulated robot
+ Detecting an obstacle and stopping the robot

### Wednesday: [SLAM & Navigation][02-slam-navigation]
**Topics**:
+ Creating a map using a lidar
+ Simultaneous Localisation and Mapping (SLAM)
+ Using move_base for navigation
+ Finding an object by navigating around a map

### Thursday: [Perception][03-perception]
**Topics**:
+ Using a camera to detect Apriltags
+ Using a real camera with ROS
+ Camera calibration
+ Fusing lidar and camera/DNN data for person detection and localisation

### Friday: [Manipulation][04-manipulation]
**Topics**:
+ Creating a Moveit configuration package
+ Moving Your robot in `rviz`
+ Using the Moveit class in a node
+ Creating a OctoMap using a depth camera

### Friday: [Mobility Plus Manipulation][05-mobility-plus-manipulation]
**Topics**:
+ How to integrate multiple ROS nodes together 
+ How to create a robot in a Gazebo world that finds and picks up as many cubes as it can

[01-sensor-integration]: https://github.com/ros-workshop/sensor-integration
[02-slam-navigation]: https://github.com/ros-workshop/slam-navigation
[03-perception]: https://github.com/ros-workshop/perception
[04-manipulation]: https://github.com/ros-workshop/manipulation
[05-mobility-plus-manipulation]: https://github.com/ros-workshop/mobility-plus-manipulation

[catkin-tools]: https://catkin-tools.readthedocs.io/en/latest/installing.html
[gh-git-flow]: https://guides.github.com/introduction/flow
[gh-ssh-keys]: https://help.github.com/articles/connecting-to-github-with-ssh/

[ros-cmakelists]: http://wiki.ros.org/catkin/CMakeLists.txt
[ros-custom-msg]: http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages
[ros-tutorials]: http://wiki.ros.org/ROS/Tutorials
