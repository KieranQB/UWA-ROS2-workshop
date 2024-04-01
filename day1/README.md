# Day 1 - The beginning, starting from scratch

## Part 1: Playing with the tools

John Vial has created a robotics challenge that sets out the tools and basic information to get a system runnning. Follow the instuction on the [github repository](https://github.com/johnny555/start-creating-robots-email)

## Part 2: Getting a glimpse of what is happening

We will now start looking up what makes up everything ROS2 and how it works starting at the basics. Run the simulation from day 5 again and in another terminal we will look at the nodes and topics that make it up. First run the command below.

```sh
ros2 node list
```

You should see a list of nodes that are currently running, lets have a closer at one of the individual nodes, run the following command from your terminal.

```sh
ros2 node info /XX
```

This command will provide you with some details about the node you are interested in for example XX

Next we will look at the topics that make up the communications between 

```sh
ros2 topic list
```

```sh
ros2 topic info /XX
```

what is the message type for XX, seach it online and workout what are the parts that make up the topic.

```sh
ros2 topic echo /XX
```

View the messages being sent onto the network.

```sh
rqt_graph
```

This shows all the nodes and topics currently running and how they are connected. It is a great way to debug a system if you don't know why two nodes aren't talking. We can even send our own commands to the robot through the command line.

```sh
ros2 topic pub /cmd_vel 
```

As you can see from this example we specify the message type that is being sent, what topic to send it to and what the values of that message should be. If we only wanted to send a message once we can add the --once tag otherwise it will continue to publish the same message a default interval. To see more with what you can do with this command run:

```sh
ros2 topic -h
```


That is it for today, the skills you learn here will help with the rest of the week in diagnosing various issues and so on.
