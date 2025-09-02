# STEP 1: Start with a Debian Bookworm base image, the same as Raspberry Pi OS
FROM arm64v8/debian:bookworm-slim

# Set shell to bash and prevent interactive prompts during installation
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# STEP 2: Install basic system dependencies for both ROS and Hailo
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    git \
    vim \
    v4l-utils \
    python3-pip \
    python3-venv \
    wget \
    curl \
    gnupg \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set up locale - required by ROS
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# STEP 3: Install ROS 2 Jazzy Jellyfish
# See official docs: https://docs.ros.org/en/jazzy/Installation/Alternatives/Debian-Packages.html
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu bookworm main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# STEP 4: Install Hailo Software Suite
RUN curl -s https://hailo-cs.s3.eu-west-2.amazonaws.com/public/Hailo-LTS/hailo.gpg.key | gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] https://hailo-cs.s3.eu-west-2.amazonaws.com/public/Hailo-LTS/raspi/ bookworm main" \
    | tee /etc/apt/sources.list.d/hailo.list > /dev/null

# STEP 5: Install all packages from all repos in one go
ARG ROS_DISTRO=jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
    hailo-all \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# --- User setup (identical to your previous file) ---
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o docker_user && \
    useradd -m -u $UID -g $GID -s /bin/bash docker_user

ARG VIDEO_GID
RUN if [ -n "$VIDEO_GID" ]; then \
        if ! getent group $VIDEO_GID > /dev/null; then groupadd -g $VIDEO_GID video; fi; \
    fi && \
    usermod -aG sudo,video docker_user

RUN echo "docker_user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker-user-sudo

COPY entrypoint.sh /home/docker_user/entrypoint.sh
RUN chown docker_user:docker_user /home/docker_user/entrypoint.sh && \
    chmod +x /home/docker_user/entrypoint.sh

USER docker_user
WORKDIR /home/docker_user

# --- Hailo Examples Setup ---
ENV DEVICE_ARCHITECTURE=HAILO8L
RUN git clone https://github.com/hailo-ai/hailo-rpi5-examples.git && \
    cd hailo-rpi5-examples && \
    ./download_resources.sh

# --- Python packages ---
RUN python3 -m pip install --user \
    'numpy<2.0' \
    opencv-python \
    # Add hailo requirements
    scipy Pillow tqdm pyyaml loguru

# --- ROS Environment Setup ---
RUN mkdir -p /home/docker_user/ros2_ws/src
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/docker_user/.bashrc
ENV PATH="/home/docker_user/.local/bin:${PATH}"
ENV PYTHONPATH="${PYTHONPATH}:/home/docker_user/hailo-rpi5-examples"

ENTRYPOINT ["/home/docker_user/entrypoint.sh"]
CMD ["bash"]