# Симуляция робота Astribot в Gazebo

Модуль содержит только часть, необходимую для симуляции: модель робота Astribot, миры Gazebo с STag маркерами и объединение облаков точек двух лидаров.

SLAM (RTAB-Map), детекция STag маркеров и навигация (Nav2) вынесены в отдельный модуль
[`navigation`](../navigation/README.md).
Оба контейнера работают с `network_mode: host` и одинаковым `ROS_DOMAIN_ID`, поэтому видят топики друг друга.

### Структура проекта

- `src/world_and_robot` - модель Astribot (`urdf_astribot/`, `meshes_astribot/`), миры (`worlds/`), модели объектов Gazebo (`gazebo_objects/`: полки, STag маркеры HD15/HD17), launch-файлы симуляции, скрипты генерации миров.
- `src/lidar_stitching` - объединение облаков точек переднего и заднего лидаров (`/lidar_front/points`, `/lidar_back/points`) в одно облако `/lidar/points_joined` в фрейме `base_link`.

Проект использует Docker для изоляции окружения ROS2 Humble и всех зависимостей.

### Запуск симуляции

После сборки образа и запуска контейнера:

```bash
xhost +local:docker
docker compose build
docker compose up -d
docker compose exec gazebo bash
```

Внутри контейнера:

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_stitching world_and_robot
source install/setup.bash
ros2 launch world_and_robot gazebo_sim.launch.py
```

По умолчанию для стабильности включён программный рендеринг. Если GPU работает нормально, его можно выключить (Gazebo работает заметно быстрее):

```bash
ros2 launch world_and_robot gazebo_sim.launch.py software_rendering:=false gazebo_gui:=true
```

**Параметры `gazebo_sim.launch.py`:**
- `world` - путь к файлу мира (по умолчанию: `worlds/astribot_hd15_hd17_walls.world`)
- `rviz_config` - конфиг RViz (по умолчанию: `rviz/gazebo_sim.rviz`)
- `gazebo_gui` - запускать GUI Gazebo (`true`/`false`)
- `software_rendering` - программный OpenGL (по умолчанию: `true`)

**Что запускается:**
- `robot_state_publisher` с моделью Astribot (все суставы зафиксированы для стабильной симуляции) и `astribot_joint_state_keeper.py`, публикующий `/joint_states`
- Gazebo и спаун робота
- `lidar_stitching_node` (через 8 секунд после старта)
- джойстик (`joy_node` + `teleop_twist_joy`) и RViz

**Основные топики симуляции:**
- `/cmd_vel` - управление базой (плагин `gazebo_ros_planar_move`)
- `/odom` и TF `odom -> base_link`
- `/lidar_front/points`, `/lidar_back/points`, `/lidar/points_joined`
- `/torso_rgbd/image_raw`, `/torso_rgbd/camera_info`

Для управления роботом можно использовать джойстик или запустить управление с клавиатуры:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Python скрипты

В пакете `world_and_robot` есть Python скрипты для генерации мира Gazebo и применения текстур.

### 1. `generate_gazebo_world.py`

Генерирует SDF мир для Gazebo с полками (`shelf_6`).

**Использование:**
```bash
cd src/world_and_robot/scripts
chmod +x generate_gazebo_world.py
./generate_gazebo_world.py --shelves 6 --marker-levels 5 --output ../worlds/generated_world.world
```

**Параметры:**
- `--shelves <N>` - количество полок (по умолчанию: 6)
- `--marker-levels <N>` - количество маркеров на каждом из 6 уровней полки (по умолчанию: 5, максимум 5)
- `--output <path>` - путь к выходному файлу `.world`

**Важно:** После создания мира обязательно пересоберите пакет:
```bash
colcon build --packages-select world_and_robot
```

### 2. `generate_gazebo_world_hd17_hd15.py`

Генерирует мир с той же раскладкой полок: по одному маркеру HD17 на угол каждой полки (6 шт.) и по три маркера HD15 на 4-м уровне каждой полки (18 шт.).

```bash
cd src/world_and_robot/scripts
./generate_gazebo_world_hd17_hd15.py --output ../worlds/generated_world_hd17_hd15.world
```

Без `--output` XML мира выводится в stdout.

### 3. `apply_stag_textures.py`

Генерирует файл `markers.material` из PNG текстур.

**Использование:**
```bash
cd src/world_and_robot/scripts
python3 apply_stag_textures.py \
    --textures ../gazebo_objects/STag_marker/materials/textures \
    --materials ../gazebo_objects/STag_marker/materials/scripts/markers.material
```

**Параметры:**
- `--textures <path>` - путь к папке с PNG текстурами маркеров
- `--materials <path>` - путь к выходному файлу `markers.material`
