# Day 3 - Robot manipulators and Image processing

Today we will be looking at two more aspects of Robotics, manipulators and controlling them using different methods and object classification.

changes to make

moveit_pose_client.cpp line 29 return result.future (same as line 39)

## Manipulators

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

## Image processing

We will start this module by building our own ROS2 node. To do this we are going to use python as the libraries are a bit easier to use. Start by creating a new ROS2 package in your source directory using the following:

```sh
ros2 pkg create imageProcessing
```

go to the src > imageProcessing > imageProcessing and create two new files 'webcam_pub.py' and 'webcam_sub.py'. You will now edit these files starting with the publisher add the following:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))

        self.get_logger().info("Publishing Video Frame")

def main(args=None):
    rclpy.init(args=args)

    image_pub = ImagePublisher()

    rclpy.spin(image_pub)

    image_pub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This creates a new publisher topic called 'video_frames'. We set a wall timer that calls the 'timer_callback' function every 100ms. Opencv2 has been initialised to capture videos on the default camera (for laptops that should be the inbuilt camera). Every time the function is called we read an image and then publish it to the topic. At the end of the function we write to the logger (if logging detail is set to info). In the main function we create the ImagePublisher object and "spin" it, which essentially keeps the node running executing its publishing command until ctrl+c is pressed to stop the node.

Now we will write the sub scriber node, again follow the below code.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolov7_package import Yolov7Detector

class ImageSub(Node):

    def __init__(self):
        super().__init__('Image_sub')

        self.subscriber = self.create_subscription(Image, 'video_frames', self.listener_callback, 1)

        self.subscriber

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info("Image recieved!")

        current_frame = self.br.imgmsg_to_cv2(msg)

        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSub()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__== '__main__':
    main()
```

This time the Node is setup with a subscriber topic and gets messages from the 'video_frames' topic. This time we don't need a wall timer to call the function, instead the subscriber will call the function "listener_callback" every time there is a new message sent to the topic. When the function is called we will display it to the screen using the opencv2 function imshow (note the cv2.waitKey(1) is needed to display the image reliably). Again we spin the node until an interrupt is called.

The last thing we need to do is tell our system that these nodes exist. Since we are using python we edit the setup.py file. Add the following XX

Once built you can run it, try testing the latency of the camera by waving your hand in front of the camera. If you notice a difference between when your hand is in front of the camera and when it is being displayed the your system is running too slow. A lagging camera will make it almost impossible to run camera based AI at real time speeds needed for robotics.

Now we can display images as a video feed it is time to add in some object classification. We are going to edit the existing subscriber node to use yolov7 which can be used natively with python3. To add in image calssification all we need to do is add the following three lines between recieving the message and displaying it.

```python
    det = Yolov7Detector(conf_thres=0.5, traced=False)
    classes, boxes, scores = det.detect(current_frame)
    current_frame = det.draw_on_image(current_frame, boxes[0], scores[0], classes[0])
```

Again rebuild the system using colcon built and run the two nodes again. This time you should see some bounding boxes around different obstacles. Try putting random objects infront of your camera to see if the model can detect it.

While it is great to be able to used a pretrained model to classify objects within our image we may want to make our own model. We will now build our own model and train it and test it to see how accurate we can predict. We will now create a nn model for identifying objects in an image using the CIFAR-10 dataset.

To start with go to the resources file and look under XX. In there you will find a script for extracting the DIFAR-10 data as well as the data files themselve.

lets create a new python file called nn.py and start by displaying some of the images. Write the following lines into your nn.py file.

```python

```


Now that we have our model we can try using it within ROS. Start by creating a new package.

```sh
ros2 pkg create
```

references: The following site was used as inspiration for this module as well as XX.