# perception-pr2
A catkin workspace in ROS where a virtual PR2 Robot with an RGBD camera perceives objects and places them 
on the appropriate dropbox.

This is my submission as partial fulfillment of [Udacity's Robotics Nanodegree](https://udacity.com/robotics).

# [WRITEUP](https://github.com/mithi/perception-pr2/blob/master/WRITEUP.md)

# Output YAML files
- [Test Scene 1](https://github.com/mithi/perception-pr2/blob/master/src/RoboND-Perception-Project/pr2_robot/scripts/output_1.yaml)
- [Test Scene 2](https://github.com/mithi/perception-pr2/blob/master/src/RoboND-Perception-Project/pr2_robot/scripts/output_2.yaml)
- [Test Scene 3](https://github.com/mithi/perception-pr2/blob/master/src/RoboND-Perception-Project/pr2_robot/scripts/output_3.yaml)

# Files 
- [Project Script](https://github.com/mithi/perception-pr2/blob/master/src/RoboND-Perception-Project/pr2_robot/scripts/project.py)
- [Resulting Model](https://raw.githubusercontent.com/mithi/perception-pr2/master/src/RoboND-Perception-Project/pr2_robot/scripts/model.sav)
- [Capture Features Script](https://github.com/mithi/perception-pr2/blob/master/src/sensor_stick/scripts/capture_features.py)
- [Training Script](https://github.com/mithi/perception-pr2/blob/master/src/sensor_stick/scripts/train_svm.py)
- [Features Script](https://github.com/mithi/perception-pr2/blob/master/src/sensor_stick/src/sensor_stick/features.py)

![Image](https://github.com/mithi/perception-pr2/blob/master/img/rviz_pr2_4.png)

# Setup
- I ran mine in ubuntu 16.04.2 with ROS full desktop version installed. It has MoveIt! Gazebo and RViz
- Make sure you have `python-pcl` as described [here](https://github.com/mithi/point-cloud-filter)
- Clone this repository 
- Install missing dependencies
```
$ cd ~/catkin_ws
$ rosdep
