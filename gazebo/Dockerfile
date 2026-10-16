ARG ROS_VER=humble

# Этап 1: Базовый ROS2 образ для симуляции
FROM osrf/ros:${ROS_VER}-desktop-full-jammy AS base

ARG APP_UID=1000

ENV \
  DEBIAN_FRONTEND=noninteractive \
  APP_USER=app                   \
  APP_UID=${APP_UID}             \
  DOCKER_GID=998                 \
  USERPASS=1

# Создание пользователя и установка sudo
# INPUT_GID=107 - GID группы input на хосте Ubuntu
RUN \
  useradd -ms /bin/bash -u ${APP_UID} ${APP_USER} && \
  echo ${APP_USER}:${USERPASS} | chpasswd && \
  usermod -aG sudo ${APP_USER} && \
  (getent group input || addgroup --gid 107 input) && \
  usermod -aG input ${APP_USER} && \
  (getent group plugdev || addgroup plugdev) && \
  usermod -aG plugdev ${APP_USER} && \
  echo ${APP_USER} 'ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
  chown -R ${APP_USER}:${APP_USER} /home/${APP_USER} && \
  addgroup --gid ${DOCKER_GID} docker && \
  addgroup ${APP_USER} docker && \
  apt-get update && \
  apt-get install -q -y sudo && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /home/${APP_USER}

USER root

# Установка системных зависимостей
RUN apt-get update && apt-get install -q -y \
  alsa-utils \
  apt-utils\
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
  tree \
  build-essential \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pytest-cov \
  python3-rosdep2 \
  python3-setuptools \
  python3-vcstool \
  libtbb-dev \
  libfreenect-dev \
  libpdal-dev \
  libusb-1.0-0-dev \
  libturbojpeg0-dev \
  libglfw3-dev && \
  pip3 install \
  numpy \
  xmlschema \
  pyyaml && \
  rm -rf /var/lib/apt/lists/*

RUN echo "if [ -f /etc/bash_completion ]; then . /etc/bash_completion; fi" >> /etc/bash.bashrc

# Этап 2: Поддержка GPU
FROM base AS gpu

USER root

ENV \
  TERM=xterm                                         \
  QT_X11_NO_MITSHM=1                                 \
  NVIDIA_VISIBLE_DEVICES=all                         \
  NVIDIA_DRIVER_CAPABILITIES=compute,utility,display

# Драйверы для Nvidia и Intel
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libgles2 \
  libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Этап 3: ROS base с зависимостями
FROM gpu AS ros_base

EXPOSE 11311

USER root

# Установка ROS2 пакетов и зависимостей, необходимых для world_and_robot и lidar_stitching
RUN apt-get update && apt-get install -q -y \
  ros-humble-rviz* \
  ros-humble-rqt* \
  ros-humble-topic-tools \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-gazebo-ros* \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-tf2-sensor-msgs \
  gazebo \
  python3-opencv \
  libopencv-dev \
  python3-scipy \
  libglib2.0-0 \
  libsm6 \
  libxext6 \
  libxrender-dev \
  libgomp1 && \
  rm -rf /var/lib/apt/lists/*

USER ${APP_USER}

ARG ROS_VER=humble

# Инициализация workspace и установка стандартных моделей Gazebo
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/setup.bash && \
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
  mkdir -p ~/ros2_ws/src && \
  mkdir -p ~/.gazebo && \
  git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models && \
  echo 'source /opt/ros/${ROS_VER}/setup.bash' >> ~/.bashrc && \
  echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc && \
  echo 'export GAZEBO_MODEL_PATH=\$HOME/.gazebo/models:/usr/share/gazebo-11/models:/opt/ros/${ROS_VER}/share/gazebo_models/models:\$GAZEBO_MODEL_PATH' >> ~/.bashrc"

# Этап 4: Финальная сборка всех пользовательских пакетов
FROM ros_base AS final

ARG ROS_VER=humble

USER ${APP_USER}

WORKDIR /home/${APP_USER}/ros2_ws/src

# Копируем исходники пользовательских пакетов
COPY --chown=${APP_USER}:${APP_USER} src/world_and_robot ./world_and_robot
COPY --chown=${APP_USER}:${APP_USER} src/lidar_stitching ./lidar_stitching

# Делаем Python скрипты исполняемыми
RUN chmod +x ./world_and_robot/scripts/*.py || true

USER root

WORKDIR /home/${APP_USER}/ros2_ws

# Инициализация rosdep и установка зависимостей
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/setup.bash && \
  rosdep init || true && \
  rosdep update && \
  apt-get update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  apt-get clean && rm -rf /var/lib/apt/lists/*"

# Сборка всех пользовательских пакетов
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/setup.bash && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Исправление прав доступа для пользователя app
RUN chown -R ${APP_USER}:${APP_USER} /home/${APP_USER}/ros2_ws

USER ${APP_USER}

WORKDIR /home/${APP_USER}/ros2_ws

CMD ["/bin/bash"]
