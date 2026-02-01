# Day 3: workspace, nodes and comms


Today we will be creating our own workspace from scratch, setting up our simulation environment in a simpler way and writing our own nodes and communication pathways


## Part 1: Setting out the work space


## Part 2: Creating a launch file


## Part 3: Creating a node with a publisher subscriber

The basic principles behind ROS2 are nodes, which contain your running code, and messaging services, which provide communication between nodes. The simplest communication method is the publisher/subscriber method. You will create a simple node that reads in the Lidar topic /scan and gets the distance to an obstacle from the central beam. You will then republish this on a new topic called /min_front_dist.


## Part 4: Create a Service

Now that you have your publisher subscriber running it is time to change it up. Now you will create a new node that takes a request to drive until the obstacle infront of it is X metres from the front of the robot using a Service.

## Part 5: Create and Action

After creating our service we can get the robot to move but we don't know what is happening while we are waiting. We will therefore create another new node that runs an action. As before the request will say how far away the robot needs to be from the wall but while it is driving the robot will give constant feedback on how far it is from the wall.