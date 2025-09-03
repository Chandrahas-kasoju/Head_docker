# Use an official ROS 2 image as a parent image
FROM debian:bookworm

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO jazzy
ENV LANG en_US.UTF-8

# Install generic requirements
RUN apt-get update && \
    apt-get install -y software-properties-common wget

# Need to create a sources.list file for apt-add-repository to work correctly:
# https://groups.google.com/g/linux.debian.bugs.dist/c/6gM_eBs4LgE
RUN echo "# See sources.lists.d directory" > /etc/apt/sources.list

RUN wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb && \
    apt install -y ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb && \
    pip install --break-system-packages vcstool psutil colcon-common-extensions

# Add Raspberry Pi repository, as this is where we will get the Hailo deb packages
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 82B129927FA3303E && \
    apt-add-repository -y -S deb http://archive.raspberrypi.com/debian/ bookworm main

# Dependencies for hailo-tappas-core
RUN apt-get update && apt-get install -y python3 ffmpeg x11-utils python3-dev python3-pip \
    gcc-12 g++-12 python-gi-dev pkg-config libcairo2-dev \
    libgirepository1.0-dev libgstreamer1.0-dev cmake \
    libgstreamer-plugins-base1.0-dev libzmq3-dev rsync git \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav \
    gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-libcamera libopencv-dev \
    python3-opencv

# Dependencies for rpicam-apps-hailo-postprocess
RUN apt-get update && apt-get install -y rpicam-apps hailo-tappas-core=3.31.0+1-1 hailo-all=4.20.0

RUN apt-get update && apt-get install -y \
    v4l-utils \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libpcl-dev \
    ros-${ROS_DISTRO}-pcl-conversions 

RUN apt update && apt install -y \
    libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev \
    libportmidi-dev libswscale-dev libavformat-dev libavcodec-dev \
    libfreetype6-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep as root
RUN rosdep init || true && rosdep update
# Create the user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o docker_user && \
    useradd -m -u $UID -g $GID -s /bin/bash docker_user

# Add the user to the sudo and video groups
ARG VIDEO_GID
ARG DIALOUT_GID
RUN if [ -n "$VIDEO_GID" ]; then \
        if ! getent group $VIDEO_GID > /dev/null; then groupadd -g $VIDEO_GID video; fi; \
    fi && \
    if [ -n "$DIALOUT_GID" ]; then \
        if ! getent group $DIALOUT_GID > /dev/null; then groupadd -g $DIALOUT_GID dialout; fi; \
    fi && \
    usermod -aG sudo,video,dialout docker_user

# Give the user password-less sudo privileges
RUN echo "docker_user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker-user-sudo

# Copy the entrypoint script and set its permissions AS ROOT
COPY entrypoint.sh /home/docker_user/entrypoint.sh
RUN chown docker_user:docker_user /home/docker_user/entrypoint.sh && \
    chmod +x /home/docker_user/entrypoint.sh

RUN apt-get update && sudo apt-get install -y \
    ffmpeg python3-virtualenv gcc-12 g++-12 python-gi-dev pkg-config \
    libcairo2-dev libgirepository1.0-dev libgstreamer1.0-dev cmake \
    libgstreamer-plugins-base1.0-dev libzmq3-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav libopencv-dev \
    python3-opencv rapidjson-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y hailo-tappas-core=3.31.0+1-1 hailo-all=4.20.0

RUN git clone https://github.com/hailo-ai/hailo-apps-infra.git /home/docker_user/hailo-apps-infra

# Run the installer
USER docker_user
WORKDIR /home/docker_user

# ### HAILO ### Set the environment variable to ensure only HAILO8L models are downloaded
ENV DEVICE_ARCHITECTURE=HAILO8L




RUN python3 -m pip install --user \
    'numpy<2.0' \
    opencv-python \
    mediapipe \
    'git+https://github.com/Chandrahas-kasoju/python-st3215.git' \
    requests \
    pygame

# Create workspace directory as the user
RUN mkdir -p /home/docker_user/ros2_ws/src
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/docker_user/.bashrc

# Set the entrypoint
ENTRYPOINT ["/home/docker_user/entrypoint.sh"]
# Set the default command
CMD ["bash"]
