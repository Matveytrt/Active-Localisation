# Active Localization

Симуляция робота Astribot, SLAM по лидару и STag маркерам (RTAB-Map) и навигация (Nav2).

Проект состоит из двух независимых модулей, каждый со своим Docker-образом и `docker-compose.yml`:

| Модуль | Что внутри | Контейнер |
|---|---|---|
| [`gazebo`](gazebo/README.md) | Gazebo, модель Astribot, миры с маркерами, `lidar_stitching` | `gazebo` |
| [`navigation`](navigation/README.md) | детектор STag, RTAB-Map с landmarks, Nav2, эксперименты и анализ детекций | `navigation` |

Модули не зависят друг от друга на уровне сборки и общаются только через ROS 2 топики: оба контейнера работают с `network_mode: host` и одинаковым `ROS_DOMAIN_ID`. Поэтому модуль `gazebo` можно заменить реальным роботом или другим симулятором, а `navigation` запускать отдельно (например, на роботе).

## Структура

```
active-localization/
├── compose.yaml          # подключает compose-файлы обоих модулей (include)
├── logs/                 # логи сборки и запуска (не попадают в git)
├── gazebo/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── src/              # world_and_robot, lidar_stitching
└── navigation/
    ├── Dockerfile
    ├── docker-compose.yml
    ├── docker/           # отдельный образ stag_ros2_enhanced и конфиги GPU/звука
    ├── maps/             # базы карт RTAB-Map (volume в контейнере)
    └── src/              # stag_ros2_enhanced, rtabmap_landmarks, tf_to_fiducial,
                          # nav2_planner_custom, stag_experiment_manager, stag_data_analyzer
```

## Требования

- Docker с Docker Compose >= 2.20.3 (нужна поддержка `include` в корневом `compose.yaml`)
- X-сервер на хосте для GUI (Gazebo, RViz)

Разрешить контейнерам доступ к X-серверу:

```bash
xhost +local:docker
```

## Запуск

Вся система из корня:

```bash
docker compose up -d --build
docker compose exec gazebo bash       # терминал симуляции
docker compose exec navigation bash   # терминал SLAM и навигации
```

Только один модуль, из его папки:

```bash
cd gazebo        # или cd navigation
docker compose up -d --build
docker compose exec gazebo bash       # или navigation
```

Собрать только один образ из корня:

```bash
docker compose build navigation
```

Остановить:

```bash
docker compose down
```

Корневой запуск и запуск из папки модуля — это разные Compose-проекты. Контейнер, поднятый из `gazebo/`, останавливайте командой `docker compose down` из той же папки.

### Логи сборки

Логи складываются в `logs/` с модулем и временем в имени, чтобы разные сборки не перезаписывали друг друга:

```bash
docker compose build --progress=plain gazebo 2>&1 | tee logs/build_gazebo_$(date +%Y%m%d_%H%M%S).log
docker compose build --progress=plain navigation 2>&1 | tee logs/build_navigation_$(date +%Y%m%d_%H%M%S).log
```

## Типовой сценарий: симуляция → SLAM → навигация

1. В контейнере `gazebo` запустить симуляцию:
   ```bash
   ros2 launch world_and_robot gazebo_sim.launch.py software_rendering:=false
   ```
2. В контейнере `navigation` запустить детекцию маркеров и SLAM:
   ```bash
   ros2 launch rtabmap_landmarks complete_system.launch.py \
       localization:=false delete_db_on_start:=true
   ```
   После объезда - Ctrl+C и экспорт поз больших маркеров из карты в `landmarks.yaml`:
   ```bash
   ros2 run rtabmap_landmarks export_landmarks_from_db
   ```
   Дальше - локализация по готовой карте: `complete_system.launch.py localization:=true`.
3. Там же (при запущенной локализации) запустить Nav2:
   ```bash
   ros2 launch nav2_planner_custom navigation_launch.py
   ```
   Цель задаётся в RViz (`rviz2`, Fixed Frame `map`) инструментом **2D Goal Pose**.

Подробности и параметры — в README модулей: [`gazebo`](gazebo/README.md), [`navigation`](navigation/README.md).

## Интерфейс между модулями

Чтобы заменить модуль `gazebo` (другим симулятором или реальным роботом), замена должна предоставлять те же топики и TF:

| Направление | Топики / TF |
|---|---|
| `gazebo` → `navigation` | `/lidar/points_joined`, `/torso_rgbd/image_raw`, `/torso_rgbd/camera_info`, `/odom`, TF `odom → base_link` и TF робота, `/clock` (в симуляции) |
| `navigation` → `gazebo` | `/cmd_vel` |
| `navigation` наружу | TF `map → odom`, `/map`, `/goal_pose` (вход Nav2), action `/navigate_to_pose` |

## Настройки DDS

Оба модуля по умолчанию используют `ROS_DOMAIN_ID=1`, `rmw_fastrtps_cpp` и `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (shared memory Fast DDS не работает между разными контейнерами). Значения можно переопределить переменными окружения перед `docker compose up`, например:

```bash
ROS_DOMAIN_ID=25 docker compose up -d
```
