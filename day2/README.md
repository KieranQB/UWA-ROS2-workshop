# Day 2: Simulation

The focus of today is getting familiar with the simulation and visualisation tools in ROS2 as well as learning how best to run your project as it gets bigger. In the second half of the day we will deploy our simulated solutions to the actual robots (There are only a handful of robots so people will need to take turns and maybe work in pairs).

## Prerequisite

Ubuntu 24.04
  - VM with .iso
    - [virtualBox](https://www.virtualbox.org/)
    - [ubuntu 24.04 .iso](https://ubuntu.com/download/desktop)
  - linux PC
    - dual boot
    - install directly
  - Using cloud based solution
    - AWS
    - Google Cloud
    - The Construct (Only 8 hours a day on free account)

Install [gazebo harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu)
Install [ros2 jazzy](https://gazebosim.org/docs/harmonic/ros_installation/)

## Part 1: The simulation

All good projects should start from a simulation, this will help your quickly develop and test your current solution in a safe environment. The simulated environment also presents the opportunity to test in hard to access regions as you define the world yourself. We will be using Gazebo Harmonic as our simulation enviornment as it works well with ROS2. In the files you have downloaded you would have seen a .sdf file and a .urdf file. The sdf file describes the world (and can also decribe the robot model in the world), a urdf on the otherhand will describe just a robot and it's connections. For this module we will start by editing the robot model and leaving the sdf file for later.

The Unified Robot Description Format (URDF) is an XML format that describes a robot's hardware, including it's chassis, linkages, joints, sensor placement, etc. Take a look at NASA's Robonaut to see what's possible with robot URDFs. We will start by editing the half finished model to look more like the pioneers and then add in the relevant sensors. start by running the below command in a terminal to determine the links and joints.

```sh
check_urdf pioneer.urdf

robot name is: pioneer3at_body
---------- Successfully Parsed XML ---------------
root Link: base_link has 3 child(ren)
    child(1):  chassis
    child(2):  p3at_front_left_axle
        child(1):  p3at_front_left_hub
            child(1):  p3at_front_left_wheel
    child(3):  top_plate
```

##### Brief description of URDF file

We break down the parts of `.urdf` briefly in this section.

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

For more information on the XML tags of URDF file, please refer to its documentation [here](http://wiki.ros.org/urdf/XML).

If you read through the provided urdf you may notice is three of the wheels are missing. Try running the system in gazebo first just to help visualise the issue. You will also need to update the <b><u>path</u></b> inside the urdf to the meshes which are visualisation files for the robot and the file path for the following command, replace the <path_to_file> section in the urdf.

```sh
#terminal 1
gz sim basic_urdf.sdf 
#terminal 2
gz service -s /world/pioneer_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "<filePath>/robots/pioneer.urdf", name: "urdf_model"'
```

You will notice the missing wheels as well as the chassis colour is incorrect (It should be red). In addition, you don't have any sensors currently attached. Try to see if you can attach the remaining wheels and update the chassis colour to be red. Try changing only one or two things at a time as it might need to be reverted. Once you have the wheels attached you can try driving the robot. In iginition gazebo click the top left button and search for `Teleop`, click on it and then scroll down to where it has been added into the menu. Change the settings to keyboard and try driving your robot (you will need to press play on the environment in the bottom left of the screen), do you notice anything strange?

The robot seems to fall to the ground and drive in a perculiar way. Try making the robot transparent by right clicking the urdf in the right hand menu and changing the properties to transparent, then add in the collision view in the same menu. What do you notice about the wheels?

They are all at right angles in the collision frame! we need to fix this in order to drive properly. Have a look under the collision tag in the link for the wheels and see if you can work it out.

### Adding a sensor

Lidar

In the pioneer.urdf add a new joint named "laser_joint" and make it a fixed joint. Link it to chassis with an origin of 0.2 in the x and 0.104 in the z. After creating the joint you will need to create a link with the name "laser_frame", have a look at the previous joints to see if you can work out how to write the link. There is a premade mesh in the meshes folder called hokuyo1.dae that you can use to visualise the lidar.

We now have a visual representation of our sensor connected to the body but we still need to setup the sensor parameters. First we define a new sensor, this needs to be within a gazebo tag that references the link we made previously. This is because only gazebo needs the sensor defined, the visualisation in ROS2 through RVIZ will just read the topic in relation to a reference frame.

```xml
<gazebo reference="laser_frame">
    <material>Gazebo/Blue</material>
    <sensor name='gpu_lidar' type='gpu_lidar'>

    </sensor>
</gazebo>
```

Next we add details about the reference frame, the topic and pose. add the following lines between the the sensor tags.

```xml
<topic>scan</topic>
<update_rate>10</update_rate>
<frame>laser_frame</frame>
```

This tells gazebo that the frame_id needs to be laser_frame, the relative position and the topic to publish on. Now we can set up the lidar tag which defines the type of lidar we want including the number of samples in a single layer and how many layers we can have. In this case we are getting 640 samples between -1.5708 and 1.5708 radians for a single layer. The samples are evenly spaced so you should be able to work out the angle of resolution. We also define the maximum and minimum range the sensors can detect, we are using a short range of 10 meters but some lidars are accurate to a few hundred meters.

```xml
<lidar>
  <scan>
      <horizontal>
      <samples>640</samples>
      <min_angle>-1.5708</min_angle>
      <max_angle>1.5708</max_angle>
      </horizontal>
  </scan>
  <range>
      <min>0.05</min>
      <max>10.0</max>
  </range>
</lidar>
<always_on>1</always_on>
```

Once you have added this in and saved it try running the system again (you might need to close your previous gazebo session). Currently you won't be able to see the lidar but Gazebo like ROS has the ability to view topics by using:

```sh
gz topic -l
```

```sh
gz topic -e --topic /lidar
```

You should see a large array of numbers with the distance values to the wall. We can also visualise the lidar in gazebo as well. Try adding the lidar visualisation by going to the top right menu and searching for `visualise lidar`. After selecting it you might need to refresh the topic in the menu and then you should see a visualisation of the lidar in gazebo.

<details>
<summary>If you are stuggling to get it working open the summary below for the full sensor code.</summary>

<br>
 
  ```xml
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
        <mesh filename="/home/user/UWA-ROS2-workshop/Resources/meshes/hokuyo1.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <gazebo reference="laser_frame">
    <sensor name="laser" type="gpu_lidar">
        <topic>scan</topic>
        <update_rate>10</update_rate>
        <frame>laser_frame</frame>

        <lidar>
        <scan>
            <horizontal>
            <samples>640</samples>
            <min_angle>-1.5708</min_angle>
            <max_angle>1.5708</max_angle>
            </horizontal>
        </scan>
        <range>
            <min>0.05</min>
            <max>10.0</max>
        </range>
        </lidar>
    </sensor>
    </gazebo>
 ```

</details>
<br>

Using your knowledge from the last step plus the following [links](https://gazebosim.org/docs/harmonic/sensors) see if you can setup the [Camera](http://sdformat.org/spec?elem=sensor) and IMU. Again you can check the values after adding the sensors using either the command line or gazebo visuals (for the camera you will need to search for `display image`).

Camera

<details>
<summary></summary>

<br>

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
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>

<link name="cam_optical_link"></link>

<gazebo reference="cam_frame">
  <sensor name="camera" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>camera/image</topic>
      <frame>cam_optical_link</frame>

      <camera>
          <horizontal_fov>1.089</horizontal_fov>

          <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
          </image>

          <clip>
              <near>0.1</near>
              <far>20.0</far>
          </clip>
      </camera>
  </sensor>
</gazebo>
```
</details>
<br>

IMU

<details>
<summary></summary>

<br>

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>imu</topic>
      <frame>base_link</frame>

      <imu>
      <angular_velocity>
          <x>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.0002</stddev>
          </noise>
          </x>
          <y>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.0002</stddev>
          </noise>
          </y>
          <z>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.0002</stddev>
          </noise>
          </z>
      </angular_velocity>

      <linear_acceleration>
          <x>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
          </noise>
          </x>
          <y>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
          </noise>
          </y>
          <z>
          <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
          </noise>
          </z>
      </linear_acceleration>
      </imu>
  </sensor>
</gazebo>
```

</details>

<br>

You now have a working simulation model ready to be used. However, the simulation can only do so much, if we want to apply our own code and drive autonomously we need to connect our simulation to ROS2. Have another read through the urdf and make sure you understand the different parts of the urdf. Try driving it around the map using the Teleop function from before, remember you can use your keyboard by selecting `keyboard` under the `Teleop` section.


### Running everything from SDF file

Instead of spawning the robot every time you want to test it you can add the following lines to the sdf file and this will spawn the robot by default.

```sh
<include>
  <uri>file://<path to robot file>/robots/pioneer.urdf</uri>
  <name>pioneer</name>
</include>
```

### Visualising with RVIZ

We now want to add the gazebo simulation topics to ROS2 so we can visualise and use the data. To do this we need to run the ros_bridge which creates a link between the gazebo topic /scan and the ROS2 topic scan. Try run the visualisation and display it in rviz2.

```sh
# Gazebo
gz sim basic_urdf.sdf

# Bridge
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# TF
ros2 run tf2_ros static_transform_publisher \
  0 0 0.1 0 0 0 base_link laser_frame

# RViz
rviz2
```

Once you are done with this try get the camera running in RVIZ2 aswell.


## Part 2: SDF

Play around with the SDF world file now. See if you can add new obstalces and layout yourself following the documentation online.