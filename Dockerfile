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
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep as root
RUN rosdep init || true && rosdep update
# Create the user

RUN git clone https://github.com/hailo-ai/hailo-apps-infra.git /home/docker_user/hailo-apps-infra

# Run the installer
# ### HAILO ### Set the environment variable to ensure only HAILO8L models are downloaded
ENV DEVICE_ARCHITECTURE=HAILO8L


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /home/docker_user

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
# Set the default command
CMD ["bash"]
