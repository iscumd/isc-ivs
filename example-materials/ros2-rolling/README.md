| Author | Email | Date |
| :----: | :----: | :----: |
| SkyPup | bamu@umich.edu | 2021.11.01 |

- [X] Setting up first docker container
- [X] Adding Nvida/Cuda RunTime 11.3.1, Ubuntu 20.04 LTS, Galactic
- [X] Adding Nvida/Cuda Developer 11.4.2, Ubuntu 20.04 LTS, Galactic
- [X] Adding Nvida/Cuda Developer 11.6.0, Ubuntu 20.04 LTS, Rolling
- [X] Add Dependencies Linux
- [_] Add ROS2 LTS Foxy
- [X] Add ROS2 Non-LTS Galactic
- [X] Add Dependencies ROS2
- [X] Add Gazebo Simulator
- [X] Add Dependencies Gazebo
- [X] Add Dependencies C++ & Python
- [X] Add Dependencies Tensorflow
#### Useful docker commands:
##### Example of Building from Dockerfile
	$ docker build -t ros2:rolling .
#### Example of Running from Dockerfile
	$ docker run --rm -it \
	  --env="QT_X11_NO_MITSHM=1" \
	  -v /tmp/.X11-unix:/tmp/.X11-unix \
	  -e DISPLAY=unix$DISPLAY \
	  ros2:rolling
#### Example of enterint Docker Container from 2nd Entry Point
	$ docker exec -it
#### Example of Running Inside Docker Container
##### Note: An Environment Varaible as configured in Dockerfile
	$ ENV ROS2_PATH=/opt/ros2_rolling
##### Use ENV to source ROS2
	$ . $ROS2_PATH/install/local_setup.bash
##### Run C++ Talker Package
	$ ros2 run demo_nodes_cpp talker
	$ ros2 run demo_nodes_cpp listener
#### Run Gazebo Demo
	$ source /opt/ros/rolling/setup.bash
	$ gazebo --verbose \
	  /opt/ros/rolling/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
	$ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 3.0}}' -1

##### [ROS2 Install Guide](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html)

##### [Gazebo ROS2 Install Guide](http://gazebosim.org/tutorials?tut=ros2_overview)
