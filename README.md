# pepper-ros-docker
This repository provides a Dockerfile that builds a relatively minimal (headless) ROS environment with driver packages for Softbank's Pepper robot. As all of the ROS packages for Pepper are pretty old and require ROS kinetic, which is not available on recent Linux distros. Thus, running a ROS kinetic docker container is an acceptable solution to run Pepper's ROS stack on more up-to-date machines.

## Installation
Simply build the Docker image locally:
```bash
git clone https://github.com/pallgeuer/pepper-ros-docker.git
cd pepper-ros-docker
docker build . -t minimal-pepper-ros-driver
```

## Usage
After having built the image, start a container as follows:
```bash
docker run -it --network host minimal-pepper-ros-driver:latest
```
The `--network host` parameter is important, as this tell the container to use the same network namespace as the host machine, which allows us to build a distributed ROS environment. 

You can now launch Pepper's ROS stack by simply calling:
```bash
roslaunch pepper_extra pepper_main.launch ip:=<PEPPER_IP>
```
Replace `<PEPPER_IP>` with the IP address of your robot (or change the default value for the parameter in the script). This will start the ROS packages for Pepper that make all sorts of data and services become available via ROS.

### A distributed ROS environment
With the minimal image we provide here, it isn't directly possible to inspect topic data with RViz or use other GUI tools inside the container, because the container is running headless, and thus GUI tools won't work. There are ways around this (nvidia-docker etc), but we suggest a different solution - simply start a `roscore` on your main machine (from which you are starting the docker image), then use that roscore inside the docker container.

Assuming you have a roscore running on your main machine prior to launching anything, export that roscore's URL in the container. In the container, run: 
```bash
export ROS_MASTER_URI=<ROSCORE-URI>
```
Replace `<ROSCORE-URI>` with whatever the value is for your roscore, e.g. `http://wtmpc810:11311`. On your main machine, you can now start tools like RViz to inspect topic data or [rqt_robot_steering](http://wiki.ros.org/rqt_robot_steering) to drive Pepper around. For RViz, you can load the `resources/pepper_config.rviz` configuration to see laser, camera and depth data.
