# FROM nvidia/cuda:11.6.2-base-ubuntu20.04 as CUDA

FROM ros:humble

# COPY --from=CUDA /usr/local/cuda /usr/local/


RUN apt update
RUN apt-get install -y -qq \
    python3-pip \
    python3-tk \
    vim git tmux tree sl htop x11-apps

RUN apt-get install -y -qq \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-microstrain-inertial-driver \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

COPY python-requirements.txt python-requirements.txt
RUN pip3 install -r python-requirements.txt


RUN echo 'source "/opt/ros/humble/setup.bash" --' >> ~/.bashrc && \
    echo 'cd rb_ws' >> ~/.bashrc && \
    echo 'colcon build --symlink-install' >> ~/.bashrc && \
    echo 'source install/local_setup.bash' >> ~/.bashrc && \
    echo 'chmod -R +x src/buggy/scripts/' >> ~/.bashrc && \
    echo 'source environments/docker_env.bash' >> ~/.bashrc 

# add mouse to tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf
