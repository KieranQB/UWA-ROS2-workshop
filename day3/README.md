# Day 3 - Robot manipulators and Image processing

Today we will be looking at two more aspects of Robotics, manipulators and controlling them using different methods and object classification.

## Manipulators

We will use a prebuilt package taken from one of the online repositories. This is a package developed by someone in the ROS community but not added to default shared ROS library. This means we need to download (or clone) it to our local repository and then build it using our colcon building tool.

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

While it is great to be able to used a pretrained model to classify objects within our image we may want to make our own model. We will now build our own model and train it and test it to see how accurate we can predict.


references: The following site was used as inspiration for this module as well as XX.