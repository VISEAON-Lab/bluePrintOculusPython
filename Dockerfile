# FROM osrf/ros:melodic-desktop-full
FROM osrf/ros:noetic-desktop-full
RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install pygccxml
RUN pip3 install castxml
RUN apt-get install -y x11-apps