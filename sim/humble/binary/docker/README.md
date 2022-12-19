| Author | Email | Date |
| :----: | :----: | :----: |
| Sim | bamu@umich.edu | 2022.12.02 |

- [X] Setting up first docker container
- [X] Adding Nvida/Cuda RunTime 11.8.0, Ubuntu 22.04 LTS, Humble

## Summary

This is a docker container for general purpose ros2 Humble development with Ign Gazebo Fortress.
It is setup to support SSH for remote development, as well as the use of modern Python and C++.   

#### Useful docker commands:
##### Building from Dockerfile
	`docker build -t isc:humble .`
#### Running from Dockerfile with GUI support
	`docker run --privileged --rm --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it isc:humble`

	|_Notes:_ privileged is needed to access AMD drivers in Gazebo. Host networking is needed so lo is shared between the container and host for X11 stuff.|
#### Running from Dockerfile with GUI support and peripheral forwarding
	`docker run --privileged --rm -v /dev:/dev --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it isc:humble`
#### Example of Running Inside Docker Container
##### Run C++ Talker Package
	`ros2 run demo_nodes_cpp talker`
	`ros2 run demo_nodes_cpp listener`
#### Run Gazebo Demo
	`ign gazebo`
	`ign gazebo diff_drive.sdf`
##### Connecting to SSH
If using the above commands, the container will be using localhost. So connections can be made to root@localhost:22, with the password 1234.

##### [ROS2 Install Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html)

##### [Gazebo ROS2 Install Guide](https://gazebosim.org/docs)