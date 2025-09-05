FROM ros:jazzy
ENV ROS_DISTRO jazzy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-colcon-clean \
    ros-${ROS_DISTRO}-common-plugins \
    ros-${ROS_DISTRO}-kinematics-interface-kdl \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-generate-parameter-library-py \
    ros-${ROS_DISTRO}-moveit-ros-control-interface \
    ros-${ROS_DISTRO}-pick-ik \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-gz-ros2-control-demos \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install ubuntu packages
RUN apt-get update -yq && apt-get install -yq \
    python3 \
    apt-utils \
    locales \
    curl \
    gnupg \
    lsb-release \
    libx11-dev \
    libxau-dev \
    libxcb1-dev \
    libxdmcp-dev \
    libxext-dev \
    lsb-release \
    pkg-config \
    byobu \
    nano \
    python3-pip \
    ssh \
    vim \
    gdb \
    udev \
    python3-pyudev \
    ipython3 \
    mesa-utils \
    autoconf \
    automake \
    libtool \
    make \
    g++ \
    unzip \
    git \
    transform3d \
    python3-scipy \
    python3-numpy \ 
    python3-cvxopt \
    python3-pandas \ 
    python3-colcon-common-extensions \
    python-is-python3 \
	python3-tk \
	python3-pytest-rerunfailures \
	python3-matplotlib \
    python3-box2d \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# RUN git clone https://github.com/xArm-Developer/xArm-Python-SDK.git && cd xArm-Python-SDK && python3 setup.py install

# install box 2d
RUN pip3 install Box2D --break-system-packages

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV QT_X11_NO_MITSHM=1

# add udev rules
RUN mkdir -p /etc/udev/rules.d
COPY ./udev_rules/* /etc/udev/rules.d/

# create a generic workplace directory
RUN mkdir -p /home/workspace/src
WORKDIR /home/workspace

# source ROS2
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# RUN chmod a+x ros_entrypoint.sh
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]