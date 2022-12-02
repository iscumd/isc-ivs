| Author | Email | Date |
| :----: | :----: | :----: |
| SkyPup | bamu@umich.edu | 2021.11.01 |

- [X] Setting up first docker container
- [X] Adding Google Tensorflow latest, Ubuntu 20.04 LTS
- [X] Adding Nvida/Cuda Developer 11.4.2, Ubuntu 20.04 LTS
- [X] Add Dependencies Linux
- [X] Add Dependencies C++ & Python
- [_] Add Dependencies Tensorflow
#### Useful docker commands:
##### Example of Building from Dockerfile
	$ docker build -t tf:tf .
#### Example of Running from Dockerfile
	$ docker run --rm -it \
	  --env="QT_X11_NO_MITSHM=1" \
	  -v /tmp/.X11-unix:/tmp/.X11-unix \
	  -e DISPLAY=unix$DISPLAY \
	  tf
#### Example of enterint Docker Container from 2nd Entry Point
	$ docker exec -it
