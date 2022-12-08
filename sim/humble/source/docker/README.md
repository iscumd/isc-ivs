| Author | Email | Date |
| :----: | :----: | :----: |
| Sim | bamu@umich.edu | 2022.12.02 |

- [X] Setting up first docker container
- [X] Adding Nvida/Cuda RunTime 11.8.0, Ubuntu 22.04 LTS, Humble

#### Useful docker commands:
##### Example of Building from Dockerfile
	$ docker build -t ros2:humble .
#### Example of Running from Dockerfile
	$ docker run --rm -it \
	  --env="QT_X11_NO_MITSHM=1" \
	  -v /tmp/.X11-unix:/tmp/.X11-unix \
	  -e DISPLAY=unix$DISPLAY \
	  ros2:humble
#### Example of enterint Docker Container from 2nd Entry Point
	$ docker exec -it
#### Example of Running Inside Docker Container
##### Note: An Environment Varaible as configured in Dockerfile
	$ ENV ROS2_PATH=/opt/ros2_humbles
##### Use ENV to source ROS2
	$ . $ROS2_PATH/install/local_setup.bash
##### Run C++ Talker Package
	$ ros2 run demo_nodes_cpp talker
	$ ros2 run demo_nodes_cpp listener
#### Run Gazebo Demo
	$ source /opt/ros/humble/setup.bash
	$ gazebo --verbose \
	  /opt/ros/humble/share/gazebo_plugins/worlds/humble_ros_diff_drive_demo.world
	$ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 3.0}}' -1

##### [ROS2 Install Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html)

##### [Gazebo ROS2 Install Guide](http://gazebosim.org/tutorials?tut=ros2_overview)