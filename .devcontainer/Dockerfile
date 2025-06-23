FROM ros:humble

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip \ 
    nano libboost-all-dev tcpreplay \
    libpcap-dev netcat iproute2
ENV SHELL /bin/bash

# Install basic development tools and ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    vim \
    tmux \
    bash-completion \
    sudo \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for the project
RUN pip3 install --no-cache-dir \
    numpy \
    pybullet \
    fastapi==0.103.1 \
    uvicorn==0.23.2 \
    websockets==11.0.3 \
    mujoco>=3.1.6 \
    "qpsolvers[daqp]>=4.3.1" \
    typing_extensions \
    loop-rate-limiters>=0.1.0 \
    "osqp>=0.6.2,<1"
    
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.

USER $USERNAME
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/tih/manipulator_teleop/ros2_ws/install/setup.bash' >> ~/.bashrc
CMD ["/bin/bash"]