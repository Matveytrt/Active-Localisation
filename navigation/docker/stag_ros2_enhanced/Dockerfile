ARG ROS_VER=humble

FROM osrf/ros:${ROS_VER}-desktop-full-jammy AS base

# Your UID should be the same AS on the main machine

ARG APP_UID=1000

ENV \
  DEBIAN_FRONTEND=noninteractive \
  APP_USER=app                   \
  APP_UID=${APP_UID}             \
  DOCKER_GID=998                 \
  USERPASS=1

# Creating a user and installing sudo
RUN \
  useradd -ms /bin/bash -u ${APP_UID} ${APP_USER} && \
  echo ${APP_USER}:${USERPASS} | chpasswd && \
  usermod -aG sudo ${APP_USER} && \
  echo ${APP_USER} 'ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
  chown -R ${APP_USER}:${APP_USER} /home/${APP_USER} && \
  addgroup --gid ${DOCKER_GID} docker && \
  addgroup ${APP_USER} docker && \
  apt-get update && \
  apt-get install -q -y sudo && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /home/${APP_USER}

USER root

RUN apt-get update && apt-get install -q -y \ 
    alsa-utils \
    apt-utils\
    cmake \
    iputils-ping \
    liburdfdom-tools \
    mc \
    bash-completion \
    mesa-utils \
    nano \
    python3-pip \
    tmux \
    htop \
    net-tools \
    curl \
    wget \
    git \
    software-properties-common \
    psmisc \
    jstest-gtk \
    tree && \
  pip3 install \
    numpy \
    xmlschema \
    pyyaml && \
  rm -rf /var/lib/apt/lists/*

RUN echo "if [ -f /etc/bash_completion ]; then . /etc/bash_completion; fi" >> /etc/bash.bashrc

FROM base AS gpu

USER root

ENV \
  TERM=xterm                                         \
  QT_X11_NO_MITSHM=1                                 \
  NVIDIA_VISIBLE_DEVICES=all                         \
  NVIDIA_DRIVER_CAPABILITIES=compute,utility,display

# Drivers for Nvidia
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 && \
  rm -rf /var/lib/apt/lists/*

# Drivers for Intel
RUN apt-get update && \
  apt-get -y install \
    libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Neccessary to make work a stack Intel/Nvidia
COPY docker_configs/glvnd_conf.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
COPY docker_configs/asound.conf /etc/asound.conf

FROM gpu AS ros_base

# Expose ports for ROS
EXPOSE 11311

USER root
RUN apt-get update && apt-get install -q -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pytest-cov \
  python3-rosdep2 \
  python3-setuptools \
  python3-vcstool && \
  rm -rf /var/lib/apt/lists/*

USER ${APP_USER}

ARG ROS_VER=humble

# Initialize ros2_ws for app
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/setup.bash && \
                  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
                  mkdir -p ~/ros2_ws/src && \
                  cd ~/ros2_ws/src && \
                  cd ~/ros2_ws/ && \
                  colcon build --symlink-install && \
                  echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"



FROM ros_base AS run
USER root

# Install ROS2 vision and image processing packages
RUN apt-get update && apt-get install -q -y \
  ros-humble-nav2-common \
  ros-humble-nav2-rviz-plugins \
  ros-humble-rviz* \
  ros-humble-rqt* \
  ros-humble-topic-tools \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-camera-info-manager && \
  rm -rf /var/lib/apt/lists/*

# Install system dependencies for OpenCV and STag
RUN apt-get update && apt-get install -q -y \
  python3-opencv \
  libopencv-dev \
  python3-scipy \
  libgl1-mesa-glx \
  libglib2.0-0 \
  libsm6 \
  libxext6 \
  libxrender-dev \
  libgomp1 && \
  rm -rf /var/lib/apt/lists/*

# Install Python packages for stag_ros2_enhanced
# NumPy must be < 2.0 for cv_bridge compatibility with ROS2 Humble

RUN pip3 install --no-cache-dir --upgrade --force-reinstall --prefer-binary \
  "numpy>=1.20.0,<2.0.0" \
  "stag-python>=0.3.0" \
  "opencv-python>=4.5.0" \
  "pyyaml>=5.4.0" \
  "reportlab>=3.6.0" \
  "Pillow>=8.0.0" \
  "scipy>=1.7.0"

# Build native STag library required by stag_ros2_enhanced (C++).
RUN cd /usr/local/lib && \
  git clone https://github.com/ManfredStoiber/stag.git && \
  cd stag && mkdir build && cd build && \
  cmake .. && make -j"$(nproc)"

USER ${APP_USER}

WORKDIR /home/app/ros2_ws

CMD ["sh", "-c", "tail -f /dev/null"]

