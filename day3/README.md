# Day 3 - Robot manipulators and Image processing

Today we will be looking at two more aspects of Robotics, some scripting tools that aren't specific to ROS but will come in handy when looking to deploy your solutions and image processing and classification used in many aspects of control.

## Part 1: ROSbags and scripts

First we will start with the last ROS specific thing in our course which is ROSbags. ROSbags allow you to capture topics while the system is running and stores them so you can view them later. This is useful in many different ways, one of the most popular usages is creating maps offline. As you can imagine SLAM takes a lot of resources to constantly update and build a map.

Go back to start creating a robot package where you mapped out the cafe. Start up the script for day 5 and then run the following in a seperate terminal

```sh
ros2 bag record -o cafeMapping -a
```

This will record all available topics (seen when running ros2 topic list). Drive around the cafe mapping out as much of the space as you can. Once finished stop the recording by pressing ctl + c, you should see a new bag file in the directory where you ran the ros bag. The old standard for ROS bag recordings was sqlite3 format, however this has changed to mcap as it gives a better compression. To record an mcap file we use XX.

You can examine the contents of a ROS bag by running the following command.

```sh
ros2 bag info cafeMapping
```

Once we have our ROSbag we can then play it back later using

```sh
ros2 bag play cafeMapping
```

This will play back the recording messages, and they are available to view with things like ros2 topic echo. Try looking at the messages coming from one of the topics in the ROS bag. This is useful as we can then run mapping functions on the collected data without running the whole system. For example, if our robot has limited processing power but we want it to use a map of the local operating area we might first record the topics relevant to mapping then move to a high performance PC to generate the map and then place the map on the limited power pc. You can also use this approach to build maps of large areas breaking the mapping into multiple sections and remove any bad scans.

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

## Part 2: Image processing

We will start this module by building our own ROS2 node. To do this we are going to use python as the libraries are a bit easier to use. Start by creating a new ROS2 package in your source directory using the following:

```sh
ros2 pkg create --build-type ament_python imageProcessing
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

The last thing we need to do is tell our system that these nodes exist. Since we are using python we edit the setup.py file. Add the following lines to the console scripts in the setup.py file.

```python
'console_scripts': [
            'img_publisher = cv_basics.webcam_pub:main',
            'img_sub = cv_basics.webcam_sub:main',
        ],
```

Once built you can run it, try testing the latency of the camera by waving your hand in front of the camera. If you notice a difference between when your hand is in front of the camera and when it is being displayed the your system is running too slow. A lagging camera will make it almost impossible to run camera based AI at real time speeds needed for robotics.

```sh
ros2 run <package_name> webcam_pub
```


```sh
ros2 run <package_name> webcam_sub
```

Now we can display images as a video feed it is time to add in some object classification. We are going to edit the existing subscriber node to use yolov7 which can be used natively with python3. To add in image calssification all we need to do is add the following three lines between recieving the message and displaying it.

```python
    det = Yolov7Detector(conf_thres=0.5, traced=False)
    classes, boxes, scores = det.detect(current_frame)
    current_frame = det.draw_on_image(current_frame, boxes[0], scores[0], classes[0])
```

Again rebuild the system using colcon built and run the two nodes again. This time you should see some bounding boxes around different obstacles. Try putting random objects infront of your camera to see if the model can detect it.

While it is great to be able to used a pretrained model to classify objects within our image we may want to make our own model. We will now build our own model and train it and test it to see how accurate we can predict. We will now create a nn model for identifying objects in an image using the CIFAR-10 dataset.

To start with go to the resources file and look under imageProcessing. In there you will find a script for extracting the DIFAR-10 data as well as the data files themselves.

lets create a new python file called nn.py and start by displaying some of the images. Write the following lines into your nn.py file.

```python
#import libraries
from data_loader import DataLoader
import random
import tensorflow as tf
from tensorflow import keras
from keras.callbacks import EarlyStopping,LearningRateScheduler
from kerastuner.tuners import RandomSearch
from kerastuner.engine.hyperparameters import HyperParameters
from keras.models import load_model,save_model,Sequential
import numpy as np
import time
import os
from sklearn.metrics import accuracy_score, confusion_matrix, ConfusionMatrixDisplay, f1_score,classification_report
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split


image_width, image_height, image_Nchannels = 32, 32, 3
class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer',
               'dog', 'frog', 'horse', 'ship', 'truck']


X_train_full, y_train_full = DataLoader.load_batch('data_batch')
X_test, y_test = DataLoader.load_batch('test_batch', Nbatches=1)


print('X_train_full.shape =', X_train_full.shape, 'data type:', X_train_full.dtype)
print('y_train_full.shape =', y_train_full.shape, 'data type:', y_train_full.dtype)
print('X_test.shape =', X_test.shape, 'data type:', X_test.dtype)
print('y_test.shape =', y_test.shape, 'data type:', y_test.dtype)


X_train, X_valid, y_train, y_valid = train_test_split(X_train_full, y_train_full,test_size=0.2,random_state=42)


print('X_train.shape =', X_train.shape, 'data type:', X_train.dtype)
print('y_train.shape =', y_train.shape, 'data type:', y_train.dtype)
print('X_valid.shape =', X_valid.shape, 'data type:', X_valid.dtype)
print('y_valid.shape =', y_valid.shape, 'data type:', y_valid.dtype)


def plot_sample_images(img_set, label_set):
    plt.figure(figsize=(18,4))
    for i in range (20):
        j=random.randint(0,len(img_set)-1)
        img=img_set[j]
        label=label_set[j]
        ax = plt.subplot(2,10,i+1)
        plt.imshow(img)
        plt.axis('off')
        ax.set_title(class_names[label])
    plt.show()


print("Training set 20 randomly sampled images:\n")
plot_sample_images(X_train, y_train)
print("Validation set 20 randomly sampled images:\n")
plot_sample_images(X_valid, y_valid)
print("Test set 20 randomly sampled images:\n")
plot_sample_images(X_test, y_test)

def Exponential_scheduler(epoch):
    initial_lrate=0.01
    if epoch < 10:
        return initial_lrate
    else:
        return initial_lrate * tf.math.exp(-0.1*epoch)


def plot_confusion_matrix(y_test, y_pred):

    cm = confusion_matrix(y_test, y_pred, normalize='true')
    cmp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=class_names)
    fig, ax = plt.subplots(figsize=(8,8))
    cmp.plot(cmap=plt.cm.Blues, ax=ax, xticks_rotation='vertical')

    cmp.im_.set_clim(0, 1)
def plot_sample_predictions(img_set, true_label_set, pred_label_set,match):
    pharse = "correctly predicted images" if match else "failure cases"
    print(f"Ten randomly sampled {pharse}:")
    plt.figure(figsize=(10,5))
    for i in range (10):
        j=random.randint(0,len(img_set)-1)
        while (true_label_set[j] == pred_label_set[j])!= match:
            j=random.randint(0,len(img_set)-1)
 
        img, true_label, pred_label=img_set[j],true_label_set[j],pred_label_set[j]
        ax = plt.subplot(2,5,i+1)
        plt.imshow(img)
        plt.axis('off')
        ax.set_title(f"Pred: {class_names[pred_label]}\nTrue: {class_names[true_label]}")
    plt.show()

callback=[]

def build_model_2():

    kernel_size=2
    kernels=64
    func = 'relu'
 
 
    CNN_dropout_rate = 0.2
    initializer = "he_normal"
    model_CNN = keras.models.Sequential([
        keras.layers.Conv2D(kernels, kernel_size, activation=func,kernel_initializer=initializer, padding="same",input_shape=[32, 32, 3]),
        keras.layers.MaxPooling2D(2),
        keras.layers.Dropout(CNN_dropout_rate),
        keras.layers.Conv2D(kernels*2, kernel_size, activation=func,kernel_initializer=initializer, padding="same"),
        keras.layers.MaxPooling2D(2),
        keras.layers.Dropout(CNN_dropout_rate),
        keras.layers.Conv2D(kernels*4, kernel_size, activation=func,kernel_initializer=initializer, padding="same"),
        keras.layers.MaxPooling2D(2),
        keras.layers.Dropout(CNN_dropout_rate),
        keras.layers.Flatten(),
        keras.layers.Dense(kernels*2,kernel_initializer=initializer),
        keras.layers.BatchNormalization(),
        keras.layers.Activation(func),
        keras.layers.Dropout(CNN_dropout_rate),
        keras.layers.Dense(kernels,kernel_initializer=initializer),
        keras.layers.BatchNormalization(),
        keras.layers.Activation(func),
        keras.layers.Dropout(CNN_dropout_rate),
        keras.layers.Dense(10, activation="softmax")
    ])
    es = EarlyStopping('val_loss', patience=3, restore_best_weights=True)
    lrs=LearningRateScheduler(Exponential_scheduler)
    callback=[es,lrs]


    model_CNN.compile(optimizer='adam',
                  loss=keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=['accuracy'])
    return model_CNN


saved_cnn_name = 'image_CNN.h5'
if os.path.exists(saved_cnn_name):
    

    model_CNN = load_model(saved_cnn_name)
    print(model_CNN.summary())# Display model architucture

    print('Training saved model with one more epoch:')
    one_history = model_CNN.fit(X_train,y_train,epochs=1,validation_data=(X_valid,y_valid),verbose=2)

else:
    # Build the CNN model from scratch 
    print(saved_cnn_name,' is not in the current directory, building the new model')
    model_CNN = build_model_2()
    print(model_CNN.summary())# Display model architucture

    print('Training the new model with 100 epochs:')
    history = model_CNN.fit(X_train, y_train, validation_data=(X_valid, y_valid), epochs=50, verbose=2, callbacks=callback)
    # Save the CNN model 
    save_model(model_CNN,saved_cnn_name)


CNN_y_pred  = np.array([np.argmax(x) for x in model_CNN.predict(X_test)])
print("Per class precision and recall metrics:")
print(classification_report(y_test, CNN_y_pred, target_names=class_names))
print(f"CNN model classification accuracy: {model_CNN.evaluate(X_test, y_test, verbose=0)[1]:.4f}\n")
F1_score = f1_score(y_test,CNN_y_pred,average='macro')
print(f"CNN model F1 score: {F1_score:.4f}\n")

plot_confusion_matrix(y_test, CNN_y_pred)


plot_sample_predictions(X_test, y_test, CNN_y_pred, match=True)
plot_sample_predictions(X_test, y_test, CNN_y_pred, match=False)

```


Now that we have our model we can try using it within ROS. Have a look at the documentation for using a [tensorflow model](). You will create 3 nodes in seperate python files, one will send images to a topic, one will read those images from the topic and run it through the nn and then publish the estimeat to another topic which will be read by the final node and published to the screen.

