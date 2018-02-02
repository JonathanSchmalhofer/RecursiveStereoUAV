FROM ubuntu:xenial
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV http_proxy 'http://proxy.muc:8080'
ENV https_proxy 'http://proxy.muc:8080'

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install additional tools
RUN apt-get update
RUN apt-get install -y git

# install ros Desktop-Full Install
RUN apt-get install -y ros-kinetic-desktop-full

# environment setup
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
