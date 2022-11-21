# Base ROS image
FROM osrf/ros:kinetic-desktop-full
MAINTAINER Philipp Allgeuer <philipp.allgeuer@uni-hamburg.de>

# Apt utilities
RUN apt-get update && apt-get install -y vim tmux wget xz-utils module-init-tools mesa-utils binutils

# ROS packages
RUN apt-get install -y ros-kinetic-pepper-robot ros-kinetic-pepper-moveit-config ros-kinetic-move-base

# Install Pepper meshes
# We have to pipe 'yes' into this to agree to the license, otherwise docker build gets stuck on this step
# We also have these debian environment params, otherwise the yes still gets stuck on the prompt in the mesh installation
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN yes | apt-get install ros-kinetic-pepper-meshes

# Custom python packages
RUN apt-get install -y python-scipy

# Get Pepper NAOqi python API (not strictly speaking required, but nice to have if one wants to quickly do API things in the container)
RUN wget --no-check-certificate https://community-static.aldebaran.com/resources/2.5.10/Python%20SDK/pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
RUN tar -xf pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
RUN rm pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
RUN echo 'export PYTHONPATH=$PYTHONPATH:/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages/' >> /ros_entrypoint.sh
RUN echo 'set +e' >> /ros_entrypoint.sh

# Get ira laser tools
RUN mkdir -p /catkin_ws/src
RUN cd /catkin_ws/src && git clone https://github.com/iralabdisco/ira_laser_tools.git
ADD resources/ira_laser_merger_pepper_params.launch /catkin_ws/src/ira_laser_tools/launch

# Add extra Pepper ROS packages
ADD pepper_extra /catkin_ws/src/pepper_extra

# Build the catkin workspace inside the container
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make'

# Modify bashrc
RUN echo 'source /ros_entrypoint.sh' >>  /root/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc
