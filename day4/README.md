# DAY 4: Scripting and going your own way

Today we will be looking at manipulators and then finishing off whatever you might have missed.

changes to make

moveit_pose_client.cpp line 29 return result.future (same as line 39)

## Part 1: Manipulators

We will use a prebuilt package taken from one of the online repositories. This is a package developed by someone in the ROS community but not added to default shared ROS library. This means we need to download (or clone) it to our local repository and then build it using our colcon building tool.

install ros2 ur_robot

## Controllers

Every motor for every joint on the robots we control needs a driver to translate our ROS commands into commands specific to the robot we are dealing with.
`ROS control` offers "controllers" which oversee a commanded trajectory and supply the drivers with low level, high frequency commands to ensure that the correct path is taken.

![ros_control_diagram](./resources/images/ros_control_diagram.png)

This is not the only way to control robot.
There are many ways in which the driver for the robotic manipulator may be interfaced with, all depending on the design of the driver.
The simulation we are running has the controllers loaded and operated by Gazebo itself, using the `gazebo_ros_control` package which is downloaded with the full desktop version of ROS.
The joint trajectories are published to Gazebo which emulates the effect of the controller.

Regardless, lets go explore what controllers are running in our simulation, and where they are defined/configured.


### Defining the Controllers

It is recommended to install an `rqt` widget called [`rqt_controller_manager`][ros-rqt-controller-manager].

```bash
sudo apt install ros-$ROS_DISTRO-rqt-controller-manager
```

Run this using the below command and let's see what got loaded when we launched our simulation.

```bash
rqt -s rqt_controller_manager
```

You should see something like this when you select the `/controller_manager` namespace in the drop down box at the top.

![controller_manager](./resources/images/controller_list.png)

Double click on these individual entries and observe the joints that they have claimed, and what type of controller it is.

You can alternatively call a particular `rosservice` call to the controller manager.
The controller manager is the node which coordinates controllers and makes sure they do not clash during run time.
Are you able to find the service to call and obtain the list without guidance?

<details><summary>Click for a walk through</summary>
<p>

---

```bash
# List the services available
rosservice list

# List the controllers
rosservice call /controller_manager/list_controllers
```

---

</p>
</details>
<br>

So, where does these come from?
Spend some time now searching through the [`universal_robot`](./Workshop/universal_robot/universal_robot) directory under [`Workshop`](./Workshop).
Can you find the config file where the controllers are defined, and when they are loaded?

<details><summary>Click to see the files</summary>
<p>

---

The configuration for the arm and gripper controllers are loaded in the very launch file we started the simulation off with ([`ur5.launch`][ur5-launch])

Lines 25 and 26 are below.

```xml
<rosparam file="$(find ur_gazebo)/controller/arm_controller_ur5.yaml" command="load"/>
<node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn arm_controller gripper_controller" respawn="false" output="screen"/>
```

* **Line 25**: shows you where you can find the controller config file.
* **Line 26**: shows us how this configuration file is used to load the controllers we want, and have them take control of the joints we want them to.

But, what about the `joint_state_controller`?

This is contained in another launch file, referenced on line 22 of the [`ur_gazebo ur5.launch`][ur5-launch] file we have been looking at.

```xml
<include file="$(find ur_gazebo)/launch/controller_utils.launch"/>
```

---

</p>
</details>
<br>


### Putting the controllers to use

It is recommended to install a simple `rqt` widget called [`rqt_joint_trajectory_controller`][ros-rqt-joint-trajectory-controller].

```bash
sudo apt install ros-$ROS_DISTRO-rqt-joint-trajectory-controller
```

Launch using the following command

```bash
rqt -s rqt_jzoint_trajectory_controller
```

![traj_controller](./resources/images/traj_controller.png)

This widget allows one to set individual joint values on the fly, and the commands are all turned into joint trajectories and sent to the controllers.
Run `rqt_graph` after hitting the red power button (it will turn green if successful) to see how this widget operates in the ROS stack.
Also, echo the commands Gazebo is receiving from this widget.

## Part 2: Doing your own thing

For now we have covered all the basics of ROS2 and you should have the skills to get started working on your own projects. From here feel free to go back and work on anything you weren't able to finish before or come up with your own driving scenario. Start by using the simulation environment and then if the pioneers are available feel free to trial your scenario.