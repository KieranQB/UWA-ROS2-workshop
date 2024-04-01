# Day 2: Mobile Robots

The focus of today is getting familiar with the simulation and visualisation tools in ROS2 as well as learning how best to run your project as it gets bigger. In the second half of the day we will deploy our simulated solutions to the actual robots (There are only a handful of robots so people will need to take turns and maybe work in pairs).

## Part 1: The simulation

All good projects should start from a simulation, this will help your quickly develop and test your current solution in a safe environment. The simulated environment also presents the opportunity to test in hard to access regions as you define the world yourself. We will be using Gazebo Ignition as our simulation enviornment as it works well with ROS2. In the files you have downloaded you would have seen a .sdf file and a .urdf file. The sdf file describes the world (and can also decribe the robot model in the world), a urdf on the otherhand will describe just a robot and it's connections. For this module we will only be editing the robot model and leaving the sdf file as is, however feel free to open up the sdf file and have a look through (ask the demonstrators if you want to know more).

The Unified Robot Description Format (URDF) is an XML format that describes a robot's hardware, including it's chassis, linkages, joints, sensor placement, etc. Take a look at NASA's Robonaut to see what's possible with robot URDFs. We will start by editing the helf finished model to look more like the pioneers and then add in the relevant sensors.

```sh
check_urdf
```

XXshow expected output

What you may notice is the back left and front right wheels are missing, first we are going to have to insert them into the urdf. Take a look at the urdf file and see if you can work out how to add the wheels in.

try running the system in gazebo

```sh
#terminal 1
ign gazebo basic_urdf.sdf 
#terminal 2
ign service -s /world/pioneer_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/quirky/Documents/dev_ws/src/p3at/robots/pioneer.urdf", name: "urdf_model"'
```

You will notice the missing wheels as well as the frame colour is incorrect

XXset material to correct colour

Adding a sensor

##### Brief description of URDF file

We break down the parts of `robot.urdf` briefly in this section.

* [`<link>`][urdf-link] element: Describes a rigid body with an inertia, visual features, and collision properties.
  * In the below snippet, we describe the base of the robot `base_link` which has a box-geometry with visual attributes like its color.

  ```XML
  <link name="base_link">
      <visual>
          <geometry>
              <box size="0.2 .3 .1"/>
          </geometry>
          <origin rpy="0 0 0" xyz="0 0 0.05"/>
          <material name="white">
              <color rgba="1 1 1 1"/>
          </material>
      </visual>
  </link>
  ```

* [`<joint>`][urdf-joint] element: Describes the kinematics and dynamics of a joint between two _links_, along with its safety limits. 
  * In the below snippet, the _joint_ description provides details on how the *base_link* and *wheel_1* links are connected. 
  * Take note of the joint type `fixed`, which means *wheel_1*
has a fixed connection with *base_link*, with all degrees-of-freedom locked.

  ```XML
  <joint name="base_to_wheel1" type="fixed">
      <parent link="base_link"/>
      <child link="wheel_1"/>
      <origin xyz="0 0 0"/>
  </joint>
  ```

For more information on the XML tags of URDF file, please refer to its
documentation [here](http://wiki.ros.org/urdf/XML).

### Adding a sensor

Lidar

In the pioneer.urdf add a new joint named "laser_joint" and make it a fixed joint. Link it to chassis with an origin of 0.2 in the x and 0.104 in the z. After creating the joint you will need to create a link with the name "laser_frame", have a look at the previous joints to see if you can work out how to write the link. There is a premade mesh in the meshes folder called hokuyo1.dae that you can use to visualise the lidar.

We now have a visual representation of our sensor connected to the body but we still need to setup the sensor parameters. First we define a new sensor, this needs to be within a gazebo tag that references the link we made previously. This is because only gazebo needs the sensor defined, the visualisation in ROS2 through RVIZ will just read the topic in relation to a reference frame.

```xml
<gazebo reference="laser_frame">
    <material>Gazebo/Blue</material>
    <sensor name='gpu_lidar' type='gpu_lidar'>"
    </sensor>
</gazebo>
```

<details>
<summary>If you are stuggling to work out how to write this have a look at the summary below.</summary>

<br>
 
  ```XML
 <!-- lidar -->
   <joint name="laser_joint" type="fixed">
     <parent link="chassis" />
     <child link="laser_frame" />
     <origin xyz="0.2 0 0.104" rpy="0 0 0"/>
   </joint>
 
   <link name="laser_frame">
     <collision>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <geometry>
         <box size="0.01 0.01 0.01"/>
       </geometry>
     </collision>
     <visual>
       <origin xyz="0 0 0" rpy="0 0 0"/> 
       <geometry>
         <mesh filename="/home/quirky/Documents/dev_ws/src/p3at/meshes/hokuyo1.dae"/>
       </geometry>
     </visual>
     <inertial>
       <mass value="1e-5" />
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
     </inertial>
   </link>
 
   <gazebo reference="laser_frame">
     <sensor name='gpu_lidar' type='gpu_lidar'>"
       <pose relative_to='laser_frame'>0 0 0 0 0 0</pose>
       <topic>lidar</topic>
       <update_rate>10</update_rate>
       <ray>
           <scan>
               <horizontal>
                   <samples>640</samples>
                   <resolution>1</resolution>
                   <min_angle>-1.396263</min_angle>
                   <max_angle>1.396263</max_angle>
               </horizontal>
               <vertical>
                   <samples>1</samples>
                   <resolution>0.01</resolution>
                   <min_angle>0</min_angle>
                   <max_angle>0</max_angle>
               </vertical>
           </scan>
           <range>
               <min>0.08</min>
               <max>10.0</max>
               <resolution>0.01</resolution>
           </range>
           <noise>
             <type>gaussian</type>
             <mean>0.0</mean>
             <stddev>0.01</stddev>
           </noise>
       </ray>
       <always_on>1</always_on>
       <visualize>true</visualize>
     </sensor>
     </gazebo>
 ```

</details>


Using your knowledge from the last step plus the following [links](https://gazebosim.org/docs/fortress/sensors) see if you can setup the [Camera](http://sdformat.org/spec?elem=sensor) and IMU.

Camera

```xml
<joint name="cam_joint" type="fixed">
    <parent link="chassis" />
    <child link="cam_frame" />
    <origin xyz="0.24 0 0.084" rpy="0 0 0"/>
  </joint>

  <link name="cam_frame">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size="0.01 0.05 0.01"/>
      </geometry>
      <material name="greenCam">
        <color rgba="0 1.0 0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="cam_optical_joint" type="fixed">
    <parent link="cam_frame" />
    <child link="cam_optical_link" />
    <origin xyz="0.3 0 0.2" rpy="-1.65 0 -1.65"/>
  </joint>

  <link name="cam_optical_link"></link>

  <gazebo reference="cam_frame">
    <material>Gazebo/Blue</material>
    <sensor name='camera' type='camera'>"
      <pose relative_to='cam_frame'>0 0 0 0 0 0</pose>
      <always_on>1</always_on>
      <visualize>true</visualize>
      <topic>camera</topic>
      <update_rate>10</update_rate>
      <camera>
        <horizontal_fov>1.089</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.5</near>
          <far>8.0</far>
        </clip>
      </camera>
    </sensor>
    </gazebo>
```

IMU


### Running in RVIZ

A launch file a has been given to you, create a new ROS2 workspace on your PC and then create a new master package. We will use this package to run all our nodes from.

When setting up our system we may run into issues where the components of the robot are not in the place we expect. We can have a look at these transforms using the following command.

```sh
ros2 run tf2_tools view_frames
```

This command listens for transform publishes on the ROS2 network and records them. It will then save these transforms to a pdf which can be viewed. Open up the view_frames file and take a look at the current transforms.

### Setting up SLAM for mapping

We have seen SLAM toolbox used yesterday, today we are going to set it up ourselves by connecting the topics to the right places. Start by downloading the [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) pacakge.

```sh
sudo apt install ros-humble-slam-toolbox
```

Try running SLAM toolbox with your current simulation now and view the topics in rqt_graph. What do you notice about the topic connections?

```sh
ros2 run sl
```

Currently SLAM toolbox is looking for lidar scans on one topic name but the lidar is publishing to a different name. To make this work we need to connect up the LiDAR topics, the naive approach would be to rewrite the nodes so that the topic name from one pacakge lines up with another. The problem with this is that you won't be able to use other peoples packages easily since you don't download the code just the compiled shared library of them. The better approach is to connect the two topics using remapping in the launch file. Let's create 

### Moving onto the real robot

Now we are going to move our code to the real pioneers. Make a copy of your current project and move it to the pioneer, once done try and launch your robot again. What do you notice?


The real robot wont run at all, this is because in the simulator we used simulated hardware now we need to use drivers to talk to the real things. We will download the following packages to setup our robot correctly.

* sick_scan_xd for the lidar
* phidget_spatial for the IMU
* luxonis depth ai for the camera

You will need to create your own launch file now similar to the one created for the simulation environment. Try seeing if you can get the system running creating your own launch file (remember to take a bitesize approach to testing that things work).
