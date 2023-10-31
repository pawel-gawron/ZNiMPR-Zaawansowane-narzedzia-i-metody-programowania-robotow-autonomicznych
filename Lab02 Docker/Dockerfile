FROM ros:noetic

RUN apt-get update && apt-get install -y vim

RUN apt-get install -y mesa-utils
RUN apt install -y ros-noetic-rviz

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /home

RUN touch file1.cpp

ENV FOO=/bar

WORKDIR ${FOO}

ADD . $FOO 
