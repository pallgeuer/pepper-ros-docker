# pepper-ros-docker
This repository provides a Dockerfile that builds a relatively minimal (headless) ROS environment with driver packages for Softbanks Pepper robot. As all of the ros packages for Pepper are pretty old, they requires ROS kinetic, which is not available on recent linux distros. Thus, running a ROS kinetic docker container is an aceptable solution to run Pepper's ROS stack on more up-to-date machines.

## Installation
Simply build the Dockerimage locally:
```bash
git clone https://github.com/frietz58/pepper-ros-docker.git
cd pepper-ros-docker
docker build . -t minimal-pepper-ros-driver
```

Alternatively, pull the image from Dockerhub:
```
docker pull frietz58/minimal-pepper-ros-driver
```

## Usage
After having build or pulled the image, start a container as follows:
```bash
sudo docker run -it --network host minimal-pepper-ros-driver:latest
```
The `--network host` parameter is critical, this tell the container to use the same network namespace as the host machine, which allows us to build a distributed ROS environment. 

You can now launch Pepper's ROS stack by simply calling `roslaunch pepper.launch ip:=<PEPPER_IP>`, replace `<PEPPER_IP>` with the IP address of you robot (or change the default value for the parameter in the script). This will start 

### A distributed ROS environment
However, you probably want to do more with Pepper, eg driving it around or looking at sensor data. With our minimal image we provide here, this isn't directly possible because it is running headless, thus RVIZ etc won't work. There are ways around this (nvidia-docker etc), but we suggest a different solution: Simply start a `roscore` on your main machine, from which you are starting the docker image. 

Assuming you have a roscore running on your main machine, before calling `pepper.launch`, export that roscore's URL in the container. In the container, run: 
```bash
export ROS_MASTER_URI=<ROSCORE-URI>
```
Replace `ROSCORE-URL>` with whatever the value for your roscore. Now, call `pepper.launch` as described above. 

On your main machine, you can now start tools like RVIZ to inspect topic data or [rqt_robot_steering](http://wiki.ros.org/rqt_robot_steering) to drive Pepper around.

For RVIZ, you can load the `pepper_config.rviz` config to see laser, camera and depth data.
