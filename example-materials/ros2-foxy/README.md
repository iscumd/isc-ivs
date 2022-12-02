| Author | Email | Date |
| :----: | :----: | :----: |
| SkyPup | bamu@umich.edu | 2021.11.01 |

- [X] Setting up first docker container
- [X] Adding Nvida/Cuda RunTime 11.3.1, Ubuntu 20.04 LTS, Galactic
- [X] Adding Nvida/Cuda Developer 11.4.2, Ubuntu 20.04 LTS, Foxy
- [X] Adding Nvida/Cuda Developer 11.4.2, Ubuntu 20.04 LTS, Galactic
- [X] Adding Nvida/Cuda Developer 11.6.0, Ubuntu 20.04 LTS, Rolling
- [X] Add Dependencies Linux
- [X] Add Dependencies ROS2
- [X] Add Gazebo Simulator
- [X] Add Dependencies Gazebo
- [X] Add Dependencies C++ & Python
- [X] Add Dependencies Tensorflow

#### Useful docker commands:
##### Example of Building from Dockerfile
	$ docker build --no-cache -t ros2:husky


#### Example of Running from Dockerfile
	$ docker run --rm -it \
	  --env="QT_X11_NO_MITSHM=1" \
	  -v /tmp/.X11-unix:/tmp/.X11-unix \
	  -v /opt/models/:/opt/models/ \
	  -e DISPLAY=unix$DISPLAY \
	  ros2:foxy

#### Example of enterint Docker Container from 2nd Entry Point
	$ docker exec -it

#### Example of Running Inside Docker Container
##### Note: An Environment Varaible as configured in Dockerfile
	$ ENV ROS2_PATH=/opt/ros2_foxy

##### Use ENV to source ROS2
	$ . $ROS2_PATH/install/local_setup.bash

##### Run C++ Talker Package
	$ ros2 run demo_nodes_cpp talker
	$ ros2 run demo_nodes_cpp listener

#### Run Gazebo Demo
	$ source /opt/ros/foxy/setup.bash

	$ gazebo --verbose \
	  /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
	
	$ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 3.0}}' -1

	$ gazebo --verbose /opt/models/duck/duck.world

	$ gazebo --verbose /opt/models/farm/farm.world

	$ gazebo --verbose /opt/models/drone/drone.world

##### [ROS2 Install Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)

##### [Gazebo ROS2 Install Guide](http://gazebosim.org/tutorials?tut=ros2_overview)

# https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/install.html#install-prerequisites
# https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md

# currently object detection is part of research models that are not available on releases, so we use master
# TODO: check v2.20 WIP object detection https://github.com/tensorflow/models/tree/v2.2.0/official/vision/detection