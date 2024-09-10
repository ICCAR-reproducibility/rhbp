FROM osrf/ros:melodic-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y python-pip \
 && rm -rf /var/lib/apt/lists/*

RUN python -m pip install h5py lindypy

RUN mkdir -p /catkin_ws/src \
 && cd /catkin_ws/src \
 && git clone --recursive https://github.com/ICCAR-reproducibility/rhbp.git

RUN cd /catkin_ws \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && catkin_make

ADD example.py .
