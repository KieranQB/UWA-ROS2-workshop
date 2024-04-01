# DAY 4: Scripting and going your own way

Today we will be looking at some scripting tools that aren't specific to ROS but will come in handy when looking to deploy your solutions.

## Part 1: ROSbags and scripts

First we will start with the last ROS specific thing in our course which is ROSbags. ROSbags allow you to capture topics while the system is running and stores them so you can view them later. This is useful in many different ways, one of the most popular usages is creating maps offline. As you can imagine SLAM takes a lot of resources to constantly update and build a map.

Go back to day 1 where you mapped out the cafe. Start up the script for part 5 and then run the following in a seperate terminal

```sh
ros2 bag record -o cafeMapping
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

Another thing we can do with ROSbags is capture data on a specific trigger. This is called a snapshot and records the last X amount of seconds before the triggering event.


bashrc


using something like terminator to start up multiple nodes (docker is better for this)


Use of ROS DOMAIN to seperate multiple ROS running environments


Use ROS XX to use a different PCs master


Make the App a desktop icon to easily start the system



Make the app a daemon to start when the PC boots up.

## Part 2: Doing your own thing

For now we have covered all the basics of ROS2 and you should have the skills to get started working on your own projects. From here feel free to go back and work on anything you weren't able to finish before or come up with your own driving scenario. Start by using the simulation environment and then if the pioneers are available feel free to trial your scenario.