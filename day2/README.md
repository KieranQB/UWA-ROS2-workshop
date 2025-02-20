# Day 2: Mobile Robots

The focus of today is getting familiar with the simulation and visualisation tools in ROS2 as well as learning how best to run your project as it gets bigger. In the second half of the day we will deploy our simulated solutions to the actual robots (There are only a handful of robots so people will need to take turns and maybe work in pairs).

## Prerequisite

Ubuntu 22.04
  - VM with .iso
    - [virtualBox](https://www.virtualbox.org/)
    - [ubuntu 22.04 .iso](https://ubuntu.com/download/desktop)
  - linux PC
    - dual boot
    - install directly
  - Using cloud based solution
    - AWS
    - Google Cloud
    - The Construct (Only 8 hours a day on free account)

Install [ign](https://gazebosim.org/docs/fortress/install_ubuntu)
Install [ros2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Part 1: The simulation

All good projects should start from a simulation, this will help your quickly develop and test your current solution in a safe environment. The simulated environment also presents the opportunity to test in hard to access regions as you define the world yourself. We will be using Gazebo Ignition as our simulation enviornment as it works well with ROS2. In the files you have downloaded you would have seen a .sdf file and a .urdf file. The sdf file describes the world (and can also decribe the robot model in the world), a urdf on the otherhand will describe just a robot and it's connections. For this module we will only be editing the robot model and leaving the sdf file as is, however feel free to open up the sdf file and have a look through (ask the demonstrators if you want to know more).

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
ign gazebo basic_urdf.sdf 
#terminal 2
ign service -s /world/pioneer_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "<filePath>/robots/pioneer.urdf", name: "urdf_model"'
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
  <ignition_frame_id>laser_frame</ignition_frame_id>
  <pose relative_to='laser_frame'>0 0 0 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
```

This tells gazebo that the frame_if needs to be laser_frame, the relative position and the topic to publish on. Now we can set up the ray tag which defines the type of lidar we want including the number of samples in a single layer and how many layers we can have. In this case we are getting 640 samples between -1.396 and 1.396 radians for a single layer. The samples are evenly spaced so you should be able to work out the angle of resolution. We also define the maximum and minimum range the sensors can detect, we are using a short range of 10 meters but some lidars are accurate to a few hundred meters.

```xml
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
```

Once you have added this in and saved it try running the system again (you might need to close your previous gazebo session). Currently you won't be able to see the lidar but Gazebo like ROS has the ability to view topics by using:

```sh
ign topic -l
```

```sh
ign topic -e --topic /lidar
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
         <mesh filename="<path_to_file>/meshes/hokuyo1.dae"/>
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
<br>

Using your knowledge from the last step plus the following [links](https://gazebosim.org/docs/fortress/sensors) see if you can setup the [Camera](http://sdformat.org/spec?elem=sensor) and IMU. Again you can check the values after adding the sensors using either the command line or gazebo visuals (for the camera you will need to search for `display image`).

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
</details>
<br>

IMU

<details>
<summary></summary>

<br>

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
  </sensor>
</gazebo>
```

</details>

<br>

You now have a working simulation model ready to be used. However, the simulation can only do so much, if we want to apply our own code and drive autonomously we need to connect our simulation to ROS2. Have another read through the urdf and make sure you understand the different parts of the urdf. Try driving it around the map using the Teleop function from before, remember you can use your keyboard by selecting `keyboard` under the `Teleop` section.


### Running in RVIZ

We will now make a new launch file from where the system can be run. Create a new ROS2 workspace on your PC (mkdir for workspace and underneath that folder a second folder called src directory) following the same layout as below (standard ROS folder layout).

```sh
- workspace/
  - src/
    - <packageName>/
      - package.xml
      - CMakeLists.txt (or setup.py)
      - src/ (where you define your nodes)
      - include/ (header files)
      - launch/ (files to launch your application)
      - config/ (yaml and other config files used in nodes and launch files)
      - maps/ (saved maps for driving)
      - models/ (all robot and world model files)
        - meshes/
        - worlds/
        - robots/
      - rviz (config for RVIZ)
  - build/
  - install/ (where you compiled code gets run from)
  - log/
```

Use the following command to build your package, dependencies are optional and not required for this exercise so remove it from your command, remember the <> are an optional name.

```sh
ros2 pkg create -build-type ament_cmake <package_name> <dependencies>
```

After creating your new package you will then need to add some resources from the resource file. First create 5 new folders "launch", "config", "robots", "worlds" and "meshes" and then copy the relevant files from the Resources directory to your new package. You will need to update the file paths again for your new folder arrangement (depending on how you set up your build system you would normally set the file path to the built location but for our purposes just putting it in the src directory is fine)

When a c++ package is building it will look at the CMakeList.txt file to determine how to build everything. At this stage we just want all our files to be added to the install directory so add the following lines to the CMakeList.txt (note for python packages the build setup is different but we won't discuss them at the moment).

```xml
install(
  DIRECTORY launch config meshes robots worlds
  DESTINATION share/${PROJECT_NAME}
)
```

Once you are happy that you have set it up you can use colcon build to build the package. (Instead of building the entire the system after every change we can select just the pacakages that we want to build using "--packages-select <packages>" after the build command.)

```sh
colcon build
```

You might have noticed that the build has failed and if you read the error you will see why.

![build_error1](../Resources/images/missingPackage2.png)

It says that there is a missing package that we need to add called `rqt_robot_steering`. This package comes from the ros repository, typicially these are `ros_XXX` for example `ros_gz_sim`. To fix this issue the first thing we can do it try to install the package.

```sh
sudo apt install ros-humble-rqt-robot-steering
```

You can see that our installs for ros packages always start with `ros-humble-` and also note that to find the right package we just add the missing package name and change any _ to -. Try building your system again and see if you get anymore issues, if you do try installing them as per above. At some point you'll come across a package that won't install.

![build_error1](../Resources/images/missingPackage1.png)

That package is p3at, the error also tells you what paths it has tried searching for the package. At this stage it is hard to know where the error is occuring but start by looking in the launch file as this is where we normally pull in other packages. If you look at line 28 of the sdf.launch.py you will see it is trying to get the shared package p3at, this is the name of my package when I built this file, you need to replace it with the name you gave your package earlier when you created it.

Now we can try launching our system using the launch file we created, first have a look at the launch file and notice what things we are trying to launch including gazebo, rviz2 and a robot state publisher. After viewing run the launch file with the following.

```sh
source install/setup.bash
ros2 launch <package_name> sdf.launch.py
```

You should see gazebo and rviz start up. Again you can check that your gazebo model is working correctly by driving it around and visualising the topics. However in RVIZ, which is the ROS side, you will notice that there are no visuals coming through and that if you echo the `/cmd_topic` there are no velocity commands while using Teleop. This is because we haven't told ROS how to talk to gazebo. For this we need a bridge that connects the two. We will now create this bridge node. Add the following lines under the robot ExecuteProcess section in the launch file.

```xml
  bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
    ,],
    output='screen',
  )
```

This will connect the lidar to the rviz by specifying the topic name we want it to publish on in ROS and the message type (you can find this by googling ros2 message type for lidar). The final part of this line is the message type from gazebo which you can find using:

```sh
ign topic -i --topic /lidar
```

The following message types can be bridged for topics (taken from the [gazebo bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) repository):

| ROS type                                       | Gazebo Transport Type               |
| ---------------------------------------------- | :------------------------------:    |
| actuator_msgs/msg/Actuators                    | ign.msgs.Actuators                   |
| builtin_interfaces/msg/Time                    | ign.msgs.Time                        |
| geometry_msgs/msg/Point                        | ign.msgs.Vector3d                    |
| geometry_msgs/msg/Pose                         | ign.msgs.Pose                        |
| geometry_msgs/msg/PoseArray                    | ign.msgs.Pose_V                      |
| geometry_msgs/msg/PoseStamped                  | ign.msgs.Pose                        |
| geometry_msgs/msg/PoseWithCovariance           | ign.msgs.PoseWithCovariance          |
| geometry_msgs/msg/PoseWithCovarianceStamped    | ign.msgs.PoseWithCovariance          |
| geometry_msgs/msg/Quaternion                   | ign.msgs.Quaternion                  |
| geometry_msgs/msg/Transform                    | ign.msgs.Pose                        |
| geometry_msgs/msg/TransformStamped             | ign.msgs.Pose                        |
| geometry_msgs/msg/Twist                        | ign.msgs.Twist                       |
| geometry_msgs/msg/TwistStamped                 | ign.msgs.Twist                       |
| geometry_msgs/msg/TwistWithCovariance          | ign.msgs.TwistWithCovariance         |
| geometry_msgs/msg/TwistWithCovarianceStamped   | ign.msgs.TwistWithCovariance         |
| geometry_msgs/msg/Vector3                      | ign.msgs.Vector3d                    |
| geometry_msgs/msg/Wrench                       | ign.msgs.Wrench                      |
| geometry_msgs/msg/WrenchStamped                | ign.msgs.Wrench                      |
| gps_msgs/msg/GPSFix                            | ign.msgs.NavSat                      |
| nav_msgs/msg/Odometry                          | ign.msgs.Odometry                    |
| nav_msgs/msg/Odometry                          | ign.msgs.OdometryWithCovariance      |
| rcl_interfaces/msg/ParameterValue              | ign.msgs.Any                         |
| ros_gz_interfaces/msg/Altimeter                | ign.msgs.Altimeter                   |
| ros_gz_interfaces/msg/Contact                  | ign.msgs.Contact                     |
| ros_gz_interfaces/msg/Contacts                 | ign.msgs.Contacts                    |
| ros_gz_interfaces/msg/Dataframe                | ign.msgs.Dataframe                   |
| ros_gz_interfaces/msg/Entity                   | ign.msgs.Entity                      |
| ros_gz_interfaces/msg/Float32Array             | ign.msgs.Float_V                     |
| ros_gz_interfaces/msg/GuiCamera                | ign.msgs.GUICamera                   |
| ros_gz_interfaces/msg/JointWrench              | ign.msgs.JointWrench                 |
| ros_gz_interfaces/msg/Light                    | ign.msgs.Light                       |
| ros_gz_interfaces/msg/ParamVec                 | ign.msgs.Param                       |
| ros_gz_interfaces/msg/ParamVec                 | ign.msgs.Param_V                     |
| ros_gz_interfaces/msg/SensorNoise              | ign.msgs.SensorNoise                 |
| ros_gz_interfaces/msg/StringVec                | ign.msgs.StringMsg_V                 |
| ros_gz_interfaces/msg/TrackVisual              | ign.msgs.TrackVisual                 |
| ros_gz_interfaces/msg/VideoRecord              | ign.msgs.VideoRecord                 |
| rosgraph_msgs/msg/Clock                        | ign.msgs.Clock                       |
| sensor_msgs/msg/BatteryState                   | ign.msgs.BatteryState                |
| sensor_msgs/msg/CameraInfo                     | ign.msgs.CameraInfo                  |
| sensor_msgs/msg/FluidPressure                  | ign.msgs.FluidPressure               |
| sensor_msgs/msg/Image                          | ign.msgs.Image                       |
| sensor_msgs/msg/Imu                            | ign.msgs.IMU                         |
| sensor_msgs/msg/JointState                     | ign.msgs.Model                       |
| sensor_msgs/msg/Joy                            | ign.msgs.Joy                         |
| sensor_msgs/msg/LaserScan                      | ign.msgs.LaserScan                   |
| sensor_msgs/msg/MagneticField                  | ign.msgs.Magnetometer                |
| sensor_msgs/msg/NavSatFix                      | ign.msgs.NavSat                      |
| sensor_msgs/msg/PointCloud2                    | ign.msgs.PointCloudPacked            |
| std_msgs/msg/Bool                              | ign.msgs.Boolean                     |
| std_msgs/msg/ColorRGBA                         | ign.msgs.Color                       |
| std_msgs/msg/Empty                             | ign.msgs.Empty                       |
| std_msgs/msg/Float32                           | ign.msgs.Float                       |
| std_msgs/msg/Float64                           | ign.msgs.Double                      |
| std_msgs/msg/Header                            | ign.msgs.Header                      |
| std_msgs/msg/Int32                             | ign.msgs.Int32                       |
| std_msgs/msg/String                            | ign.msgs.StringMsg                   |
| std_msgs/msg/UInt32                            | ign.msgs.UInt32                      |
| tf2_msgs/msg/TFMessage                         | ign.msgs.Pose_V                      |
| trajectory_msgs/msg/JointTrajectory            | ign.msgs.JointTrajectory             |
| vision_msgs/msg/Detection2D                    | ign.msgs.AnnotatedAxisAligned2DBox   |
| vision_msgs/msg/Detection2DArray               | ign.msgs.AnnotatedAxisAligned2DBox_V |
| vision_msgs/msg/Detection3D                    | gz::msgs::AnnotatedOriented3DBox    |
| vision_msgs/msg/Detection3DArray               | gz::msgs::AnnotatedOriented3DBox_V  |

Add the bridge variable to the launch description at the end of the launch file. Try rebuilding (`colcon build`) your package again and the launching it again. In RVIZ add the lidar topic to RVIZ by clicking `Add` on the left and click `By Topic` in the popup menu, you should see a topic there called `/lidar`, add this to RVIZ, you will need to run the simulation and you will notice that it still isn't visible but why is that? This is because ROS2 (and by extension RVIZ) use frame id's and transforms to position everything in the world relative to a fixed frame. In this case we need to select the fixed frame in RVIZ (change it to base_link in the left hand menu) and  update the current frame id for this topic from gazebo so that it has a connection to the base_link. Before ending your simulation save the RVIZ config by going file > save config.

When setting up our system we may run into issues where the components of the robot are not in the place we expect or not showing up as there is no link between the transforms. We can have a look at these transforms using the following command.

```sh
ros2 run tf2_tools view_frames
```

This command listens for transform publishes on the ROS2 network and records them. It will then save these transforms to a pdf which can be viewed. Open up the view_frames file and take a look at the current transforms. You should see the transforms are not connected.

We can also echo a given topic once to see what the frame_id currently is.

```sh
ros2 topic echo /lidar --once
```

scroll to the header in the topic and check the frame_id. Now that we know what the problem is it is time to fix it. There are several ways to fix this, we could publish a static transform in the launch file so there is a link between the two frames however there is a simpler way. Go back to your pioneer urdf and add the following line in under the <sensor> tag.

```xml
<ignition_frame_id>laser_frame</ignition_frame_id>
```

save and rebuild your system and then run it again, you should now see that the lidar comes up when you add it to RVIZ. If you want to view your robot model you can again add it from the `Add` button in the left corner and just click robot model. In the RVIZ menu on the left under robot model there will be an option called Description Topic, select `/robot_description`.

We now need to add in a few more topic to bridge including:

- imu
- camera
- odom
  - this will also require a bridge for the tf2 messages
  - this and the tf2 messages will have a topic name /model/pioneer3at_body/odometry since we didn't define it in gazebo
- cmd_vel

See if you can add these in yourself, with the command velocity and odom you will also need to add in some topic remappings. To do this add the following lines after the `output=` line.

```xml
  remappings=[('/cmd_vel','/cmd_vel'),
    ('/model/pioneer3at_body/odometry','/odom'),
    ('/model/pioneer3at_body/tf','/tf')
  ]
```

Remapping allows you to easily change a nodes subscribing or publishing name so they all line up.

If you are still getting transform errors you might need to add in a static transform between the gazebo footprint and the base footprint

```python
pioneer_base_fp_link_tf = Node(package='tf2_ros', 
                                executable='static_transform_publisher', 
                                name='base_fp_linkTF', 
                                output='log', 
                                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  'pioneer3at_body/base_footprint', 'base_footprint'])
```

<details>
<summary>Example of the final bridge node</summary>
<br>

```python
bridge = Node(
  package='ros_gz_bridge',
  executable='parameter_bridge',
  arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
              '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
              '/model/pioneer3at_body/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
              '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
              '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
              '/model/pioneer3at_body/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
              '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',],
  output='screen',
  remappings=[('/cmd_vel','/cmd_vel'),
              ('/model/pioneer3at_body/odometry','/odom'),
              ('/model/pioneer3at_body/tf','/tf')
  ]
)
```

</details>

<br>
Now that the robot model is complete and 


### Setting up SLAM for mapping

We have seen SLAM toolbox used yesterday, today we are going to set it up ourselves by connecting the topics to the right places. Start by downloading the [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox) pacakge.

```sh
sudo apt install ros-humble-slam-toolbox
```

Once installed we will add slam to our launch file, add the following lines to the launch file (remember to change the package name) and add the node to the launch description.

```python
slam_toolbox = Node( 
  package='slam_toolbox', 
  executable='async_slam_toolbox_node', 
  parameters=[
        get_package_share_directory('<package_name>') + '/config/mapping.yaml'
  ], 
  output='screen',
)
```

This time we are loading in a config file using the get_package_share_directory. Have a look at the mapping yaml under the config file and you will see all the configuration settings for SLAM toolbox.

Try running running your launch file again with SLAM toolbox with your current simulation now and view the topics in rqt_graph. What do you notice about the topic connections?

Currently SLAM toolbox is looking for lidar scans on one topic name but the lidar is publishing to a different name. To make this work we need to connect up the LiDAR topics, the naive approach would be to rewrite the nodes so that the topic name from one pacakge lines up with another. The problem with this is that you won't be able to use other peoples packages easily since you don't download the code just the compiled shared library of them. The better approach is to connect the two topics using remapping in the launch file. Go back to your slam toolbox node in your launch file and add the remappings.

For autonomously driving the robot we are going to use the nav2 plugin, this plugin is huge and has many additional features that we aren't going to go into today.In addition, we will need [navigation2]() for path planning and driving. These can be installed using the follow package.

```sh
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Moving onto the real robot

Now we are going to move our code to the real pioneers. Make a copy of your current project and move it to the pioneer, once done try and launch your robot again. What do you notice?

The robot is still using the simulated hardware, this is because iwe haven't told the system to use drivers to talk to the real devices. We will download the following packages to setup our robot correctly.

* joy for controller
* joy teleop for going between controller and velocity commands
* sick_scan_xd for the lidar
* phidget_spatial for the IMU
* luxonis depth ai for the camera

You will need to create your own launch file now similar to the one created for the simulation environment. Try seeing if you can get the system running creating your own launch file (remember to take a bitesize approach to testing that things work).
