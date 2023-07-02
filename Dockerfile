# FROM osrf/ros:melodic-desktop-full
FROM osrf/ros:noetic-desktop-full
RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install pygccxml
RUN pip3 install castxml




# Dev/Debug
RUN apt-get install -y x11-apps
RUN apt-get install -y git







WORKDIR /home
RUN mkdir -p /home/catkin_ws/src/


# RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
RUN echo "export ROS_NAMESPACE=sparus2" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://sparus2:11311/" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311/" >> ~/.bashrc
# RUN echo "export ROS_IP=localhost" >> ~/.bashrc
RUN echo "export ROS_IP=127.0.0.1" >> ~/.bashrc




# catin make
# WORKDIR /home/user/catkin_ws/src
# TODO: fix this
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash'
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash && catkin_make'
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash && /opt/ros/noetic/bin/catkin_make'


# source ROS
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# source catkin ws
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc
# RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
