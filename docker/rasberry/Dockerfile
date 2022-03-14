# Use the official image as a parent image.
#FROM ubuntu:18.04
FROM nvidia/cuda:10.1-cudnn7-devel
# Set the working directory.
WORKDIR /
# Copy the file from your host to your current location.
#COPY package.json .

# Run the command inside your image filesystem.
COPY ./keyboard /etc/default/keyboard

RUN mkdir /run/sshd
RUN apt update
RUN apt install -y curl lsb-release gnupg2 apt-transport-https locales
RUN echo "en_GB.UTF-8 UTF-8" | tee -a /etc/locale.gen
RUN locale-gen
RUN curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
#RUN echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list
RUN apt update
RUN apt upgrade -y
RUN apt install -y python-rosdep python-catkin-tools htop
#caffe dependencies
RUN DEBIAN_FRONTEND=noninteractive apt install -y libatlas-base-dev libgflags-dev libprotobuf-dev libopencv-dev liblmdb-dev libleveldb-dev libsnappy-dev
WORKDIR /home/rasberry
#COPY ./libcudnn7_7.4.2.24-1+cuda9.0_amd64.deb /home/rasberry
#RUN dpkg -i libcudnn7_7.4.2.24-1+cuda9.0_amd64.deb
#RUN rm /home/rasberry/libcudnn7_7.4.2.24-1+cuda9.0_amd64.deb

#COPY ./libcudnn7-dev_7.4.2.24-1+cuda9.0_amd64.deb /home/rasberry
#RUN dpkg -i libcudnn7-dev_7.4.2.24-1+cuda9.0_amd64.deb
#RUN rm /home/rasberry/libcudnn7-dev_7.4.2.24-1+cuda9.0_amd64.deb

#openpose dependencies
RUN apt install -y libgoogle-glog-dev libiberty-dev binutils-dev guile-2.2-dev libgearman-dev python-igraph
#RASberry dependencies
RUN apt install -y ros-melodic-people-msgs ros-melodic-robot-pose-publisher ros-melodic-strands-navigation-msgs ros-melodic-topological-navigation ros-melodic-rosduct ros-melodic-bayes-people-tracker ros-melodic-strands-executive-msgs ros-melodic-persistent-topics ros-melodic-contact-monitor ros-melodic-topological-utils ros-melodic-interactive-markers python-matplotlib ros-melodic-rosbridge-server ros-melodic-map-server ros-melodic-xacro ros-melodic-robot-state-publisher ros-melodic-thorvald-twist-mux ros-melodic-ira-laser-tools ros-melodic-robot-localization ros-melodic-amcl ros-melodic-mapviz ros-melodic-mapviz-plugins ros-melodic-realsense2-camera ros-melodic-teb-local-planner ros-melodic-ros-numpy ros-melodic-moveit-core ros-melodic-gmapping rviz ros-melodic-joy ros-melodic-controller-manager ros-melodic-fake-localization ros-melodic-cob-generic-can ros-melodic-nmea-navsat-driver ros-melodic-tuw-multi-robot-msgs ros-melodic-xsens-driver ros-melodic-control-msgs ros-melodic-rviz ros-melodic-marvelmind-nav ros-melodic-gazebo-plugins ros-melodic-rqt-gui ros-melodic-rqt-gui-py ros-melodic-joint-limits-interface ros-melodic-gazebo-ros-control ros-melodic-urg-node libdxflib-dev python-sklearn ros-melodic-thorvald-description ros-melodic-thorvald-base ros-melodic-polytunnel-navigation-actions ros-melodic-thorvald-example-robots ros-melodic-roboticsgroup-gazebo-plugins ros-melodic-rqt-logger-level ros-melodic-thorvald-simulator ros-melodic-thorvald-gazebo-plugins


RUN rosdep init
RUN groupadd -g 1000 rasberry
RUN useradd --no-log-init -d /home/rasberry -p "rvhUTLH21UBz." -u 1000 -g 1000 rasberry
RUN adduser rasberry sudo
WORKDIR /home/rasberry
ENV HOME /home/rasberry
RUN chown rasberry:rasberry /home/rasberry
USER rasberry

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# do the following when not using the volume below
#WORKDIR /home/rasberry/catkin_ws/src
#RUN source /opt/ros/melodic/setup.sh
#WORKDIR /home/rasberry/catkin_ws
#RUN catkin init
#RUN catkin build
#WORKDIR /home/rasberry/catkin_ws/src
#COPY cp -r ./rasberry-navigation-actions /home/rasberry/catkin_ws/src/
#RUN git clone https://github.com/LCAS/RASberry.git
#RUN rosdep update
# Inform Docker that the container is listening on the specified port at runtime.


# Run the specified command within the container.
#CMD [ "npm", "start" ]
#WORKDIR /home/rasberry
#WORKDIR /home/rasberry/catkin_ws/src/RASberry
#CMD ["launch.sh"]

# Copy the rest of your app's source code from your host to your image filesystem.
#COPY . .
VOLUME /home/rasberry/
