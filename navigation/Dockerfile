ARG ROS_VER=humble

# Этап 1: Базовый образ ROS 2 desktop-full, пользователь app и системные утилиты
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
  cmake \
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

# Копирование конфигураций для GPU (из stag_ros2_enhanced)
COPY docker/stag_ros2_enhanced/docker_configs/glvnd_conf.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
COPY docker/stag_ros2_enhanced/docker_configs/asound.conf /etc/asound.conf

# Этап 3: ROS base с зависимостями
FROM gpu AS ros_base

EXPOSE 11311

USER root

# Установка ROS2 пакетов и зависимостей
# Включая готовые пакеты rtabmap и rtabmap_ros из репозитория
# (без Gazebo и gazebo_ros — контейнер не зависит от симулятора)
RUN apt-get update && apt-get install -q -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-nav2-rviz-plugins \
  ros-humble-rviz* \
  ros-humble-rqt* \
  ros-humble-topic-tools \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-position-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-rqt-joint-trajectory-controller \
  ros-humble-rtabmap \
  ros-humble-rtabmap-ros \
  python3-opencv \
  libopencv-dev \
  python3-scipy \
  libglib2.0-0 \
  libsm6 \
  libxext6 \
  libxrender-dev \
  libgomp1 && \
  rm -rf /var/lib/apt/lists/*

# Установка Python пакетов для stag_ros2_enhanced
# NumPy должен быть < 2.0 для совместимости с cv_bridge и ROS2 Humble
RUN pip3 install --no-cache-dir --upgrade --force-reinstall --prefer-binary \
  "numpy>=1.20.0,<2.0.0" \
  "stag-python>=0.3.0" \
  "opencv-python>=4.5.0" \
  "pyyaml>=5.4.0" \
  "reportlab>=3.6.0" \
  "Pillow>=8.0.0" \
  "scipy>=1.7.0"

# Python пакеты для stag_data_analyzer (графики plotly и их экспорт в PNG через kaleido).
# kaleido 0.2.1 содержит встроенный Chromium; kaleido>=1.0 требует отдельно установленный Chrome.
RUN pip3 install --no-cache-dir --prefer-binary \
  "plotly>=5.0.0,<6.0.0" \
  "kaleido==0.2.1"

# Сборка нативной библиотеки STag, необходимой C++ детектору stag_ros2_enhanced
RUN cd /usr/local/lib && \
  git clone https://github.com/ManfredStoiber/stag.git && \
  cd stag && mkdir build && cd build && \
  cmake .. && make -j"$(nproc)"

USER ${APP_USER}

ARG ROS_VER=humble

# Инициализация workspace (без установки моделей Gazebo и переменных GAZEBO_MODEL_PATH)
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/setup.bash && \
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
  mkdir -p ~/ros2_ws/src && \
  echo 'source /opt/ros/${ROS_VER}/setup.bash' >> ~/.bashrc && \
  echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"

# Этап 4: Финальная сборка всех пользовательских пакетов
FROM ros_base AS final

ARG ROS_VER=humble

USER ${APP_USER}

WORKDIR /home/${APP_USER}/ros2_ws/src

# Копируем исходники только ваших пользовательских пакетов
# rtabmap и rtabmap_ros уже установлены из apt
COPY --chown=${APP_USER}:${APP_USER} src/stag_ros2_enhanced ./stag_ros2_enhanced
COPY --chown=${APP_USER}:${APP_USER} src/stag_experiment_manager ./stag_experiment_manager
COPY --chown=${APP_USER}:${APP_USER} src/stag_data_analyzer ./stag_data_analyzer
COPY --chown=${APP_USER}:${APP_USER} src/rtabmap_landmarks ./rtabmap_landmarks
COPY --chown=${APP_USER}:${APP_USER} src/tf_to_fiducial ./tf_to_fiducial
COPY --chown=${APP_USER}:${APP_USER} src/nav2_planner_custom ./nav2_planner_custom

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
