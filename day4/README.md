# Day 4: Scripting, control and lifecycle


Today we will be looking at two more aspects of Robotics, some scripting tools that aren't specific to ROS but will come in handy when looking to deploy your solutions and image processing and classification used in many aspects of control.

## Part 1: ROSbags and scripts

First we will start with the last ROS specific thing in our course which is ROSbags. ROSbags allow you to capture topics while the system is running and stores them so you can view them later. This is useful in many different ways, one of the most popular usages is creating maps offline. As you can imagine SLAM takes a lot of resources to constantly update and build a map.

Go back to start creating a robot package where you mapped out the cafe. Start up the script for day 5 and then run the following in a seperate terminal

```sh
ros2 bag record -o cafeMapping -a
```

This will record all available topics (seen when running ros2 topic list). Drive around the cafe mapping out as much of the space as you can. Once finished stop the recording by pressing ctl + c, you should see a new bag file in the directory where you ran the ros bag. The old standard for ROS bag recordings was sqlite3 format, however this has changed to mcap as it gives a better compression.

You can examine the contents of a ROS bag by running the following command.

```sh
ros2 bag info cafeMapping
```

Once we have our ROSbag we can then play it back later using

```sh
ros2 bag play cafeMapping
```

This will play back the recording messages, and they are available to view with things like ros2 topic echo. Try looking at the messages coming from one of the topics in the ROS bag. This is useful as we can then run mapping functions on the collected data without running the whole system. For example, if our robot has limited processing power but we want it to use a map of the local operating area we might first record the topics relevant to mapping then move to a high performance PC to generate the map and then place the map on the limited power pc. You can also use this approach to build maps of large areas breaking the mapping into multiple sections and remove any bad scans. There are other tools like (foxglove studio)[https://foxglove.dev/]

Another thing we can do with ROSbags is capture data on a specific trigger. This is called a snapshot and records the last X amount of seconds before the triggering event, where X is a specified buffer of time.

```sh
ros2 bag record -d 30 --snapshot-mode -a
```

In our code we then add a client ready to request a snapshot. Then somewhere further in the code we send a request to the client requesting a snapshot be taken and wait for it to return a result.

```python
self.snapshot_client = self.create_client(Snapshot, '/rosbag2_recorder/snapshot')

...

snapshot_req = Snapshot.Request()
self.future = self.snapshot_client.call_async(snapshot_req)
while(self.future.result() == False):
    self._timer_cb()
```

If you are working in a team you might end up working on the same robot at the same time but don't want to interfere with each other. We can set the ROS_DOMAIN id to a unique number so that each users comms are in a different space.

```sh
export ROS_DOMAIN_ID=5
```

Now anyone not using the ROS DOMAIN of 5 won't be able to send or recieve message on my ROS stack.


The following section is specific to Ubuntu as most ROS systems run on Ubuntu. The following can make your applications more user friendly.

When you open a terminal or an executable that uses bash the .bashrc script, which lives in your home directory, runs adding functionality to your terminal. There are many things you might want to add to this to make your life easier. First you might want to add sourcing ROS to your terminal, add the following line to your .bashrc

```sh
source /opt/ros/humble/setup.bash
```

Next you might get sick of writing heaps of lines to start up your application. We can create an alias or shortcut to run your application. Add the following lines to your .bashrc.

```sh
alias qRun='source /opt/ros/humble/setup.bash;source /home/quirky/Documents/dev_ws/install/setup.bash;ros2 launch <your_package> sdf.launch.py'
```

Since the .bashrc only runs when you open a terminal you either need to start a new terminal or source the .bashrc file before your can use your new alias. Once you have sourced the .bashrc try runnning it using `qRun` in your terminal.

We can also set up environment variables from the .bashrc, for more details on this please look at the documentation online.

Another useful tool is tmux. This application allows users to run multiple terminals side by side which is great if you want to view multiple running terminals at once which often happens in ROS. Another benefit of tmux is that it runs in the background, this means that if you are disconnected from a remote terminal or need to close a terminal for some reason your running script will continue in the background, look at the [tmux](https://github.com/tmux/tmux/wiki/Getting-Started) documentation and try running the basic [talker and listener nodes](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). After that close the tmux window with the cross in the top right and then connect to the tmux session again. You should see that the talker and listener continued to run in the background.

In some cases it might be convenient to run your system with a desktop icon. To do this first we create a new text file in the desktop directory with the .desktop file extension. Now open the file up and edit the content with the following.

```sh
[Desktop Entry]
Type=Application
Terminal=true
Name=ROS_Node
Icon=/home/quirky/Downloads/ros-logo.jpg
Exec=bash -c "source /opt/ros/humble/setup.bash;source /home/quirky/Documents/dev_ws/install/setup.bash;ros2 launch p3at sdf.launch.py"
Categories=Application
```
The first line specifies we want to make a desktop shortcut to run something. Terminal set to true means the terminal will open when the application is run. Name and Icon are aesthetics for users. Exec is what runs when the Icon is clicked. In this case we open a new bash terminal and source our ros and local directories, and then run the ros application.

Sometimes it is better to have a ROS system start up without needing to push anything, to do this we use systemd (daemons) to run code on start up. Let's first create a script to run our ros launch file. Create a new file called rosRun.sh and add the following lines to it.

```sh
#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/quirky/Documents/dev_ws/install/setup.bash
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

then we need to make it an executable file.

```sh
chmod a+x rosRun.sh
```

Try running your code now using `./rosRun.sh`. If it runs successfully then you can now add the script to the system startup.

Now you need to create a new file in /etc/systemd/system/ with the extension .service. After that edit the file and add the following lines.

```sh
[Unit]
Description=My Script
After=network.target
[Service]
ExecStart=/path/to/script
[Install]
WantedBy=default.target
```

next we need to install the script into the list of available service and enable the service. The first command will reload the list of available services running.

```sh
sudo systemctl daemon-reload
sudo systemctl enable <Name_of_your_service>.service
sudo systemctl start <Name_of_your_service>.service
```


## Part 2: Lifecycles

In more advance systems a nodes lifecycle can be managed to activate and deactivate the node. This is useful if you are changing between control stratergies or if a driver fails and needs to be restarted without breaking down the entire software stack. We will look at how to do this with our simple example from yesterday. A lifecycle node (a variation of the original node) acts like a state machine with set states that it can transition to, the transitions will create or destroy resources related to the node.

For this we will create a new node that inherits from LifecycleNode as oppose to Node. We will initialise it with a publishing rate and speed as we will use it to talk to cmd_vel. At each point in the state machine we will also print out the current state of the node.

```sh
#!/usr/bin/env python3
import rclpy
# from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from geometry_msgs.msg import Twist

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("publisher_example")
        self.get_logger().info("Lifecycle Example to control cmd_vel!")
        self.get_logger().info("IN constructor")
        self.publish_frequency_ = 1.0
        self.current_speed_ = 0.0

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.current_speed_
        self.cmd_vel_publisher_.publish(msg)
```

Now we will add the functions for each transition state. You will need to define an action for each of the state changes:

- on_configure = Create a publisher topic and wall timer
- on_activate = set the current speed to 0.5 and start sending messages
- on_deactivate = stop sending messages, set the speed to 0.0 and send once to ensure vehicle is stopped.
- on_cleanup = destroy the publisher and the timer
- on_shutdown = same as cleanup
- on_error = same as cleanup

For each state also run the logger to specify which state it is currently in.

<details>
<summary>If you are stuggling to get it working open the summary below for the full sensor code.</summary>

<br>

```sh
#!/usr/bin/env python3
import rclpy
# from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from geometry_msgs.msg import Twist

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("publisher_example")
        self.get_logger().info("Lifecycle Example to control cmd_vel!")
        self.get_logger().info("IN constructor")
        self.publish_frequency_ = 1.0
        self.current_speed_ = 0.0
        
    # Create ROS2 communications, connect to HW    
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.cmd_vel_publisher_ = self.create_lifecycle_publisher(Twist, "cmd_vel", 10)
        self.send_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_cmd_vel)
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.current_speed_ = 0.5
        self.send_timer_.reset()
        return super().on_activate(previous_state)
    
    # Deactivate/Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.send_timer_.cancel()
        self.current_speed_ = 0.0
        self.publish_cmd_vel()
        return super().on_deactivate(previous_state)

    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        self.destroy_timer(self.send_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    # Shutdown the lifecycle node and finalise.
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        self.destroy_timer(self.send_timer_)
        return TransitionCallbackReturn.SUCCESS

    # Handle errors. Usually not used and instead using try: except
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        self.destroy_timer(self.send_timer_)
        # return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.current_speed_
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

```
</details>
<br>


Don't forget to add a main function and start the node.

```sh
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

```

No run up your new node and try transitioning between states from command line.

```sh
ros2 lifecycle set <node_name> <state>
```


## Part 3: Control

ROS introduces quite a bit of overhead that can reduce the timing of critical features. While this is not always a problem there are certain systems that require real time operations. In order to achieve this ros2 introduces another concept called ros2_control. While it can be confusing this setup does allow for real time operations and is reserved for select special features of the robot. We will update our pioneer urdf to incoporate ros2_control now.

To start off we need to describe the controller in the robot urdf by adding a new joint for the left and right as well as a hardware plugin as follows

```sh
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

next we need to add in a description of how the the simulation and ros need to interact, inside the urdf add the following:

```sh
<gazebo>
  <plugin name="gz_ros2_control" filename="libgz_ros2_control.so">
    <parameters>$(find pioneer_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

then under src/<package_name>/config create a new file called controllers.yaml and add the following details.

```sh
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.39
    wheel_radius: 0.095

    base_frame_id: base_link
    odom_frame_id: odom
    publish_tf: true
    use_stamped_vel: false
```

Now when you go to run your system you will need to start the new controllers.

```sh
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
```

you can double check the state of the controllers by running:

```sh
ros2 control list_controllers
```

If the above controllers do not say ACTIVE then something has gone wrong.

Double check the transforms are publishing correctly by viewing the frames, and try driving the robot using the teleop.
