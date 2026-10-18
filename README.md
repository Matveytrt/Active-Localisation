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

## Active-режим: навигация с учётом наблюдаемости маркеров

Два нативных плагина Nav2 из пакета [`al_nav2_plugins`](navigation/src/al_nav2_plugins/README.md) (ADR-001) меняют только то, *как* робот едет, а не куда: поведенческое дерево, recovery и action `/navigate_to_pose` остаются базовыми.

| Плагин | Где подключается | Что делает |
|---|---|---|
| `al_nav2_plugins/LocalizabilityLayer` | слой `loc_layer` в `global_costmap` (перед `inflation_layer`) | **рычаг маршрута**: добавляет к стоимости клеток `c_max·(1 − min(s*/s_ref, 1))^p`, где `s*` - лучшая по 36 курсам оценка наблюдаемости маркеров из клетки |
| `al_nav2_plugins/LocalizationAwarePlanner` | `planner_server`, плагин `GridBased` (внутри Theta*) | **рычаг курса**: после Theta* подбирает курс вдоль пути (Витерби по 36 курсам) и пишет его в позы пути |

Для контроллера MPPI в active-режиме включён `PathAlignCritic.use_path_orientations`, а `PathAngleCritic` и `PreferForwardCritic` выключены (иначе они спорят с курсами из пути). Все отличия active от baseline лежат в одном файле параметров, [`nav2_params_active.yaml`](navigation/src/nav2_planner_custom/config/nav2_params_active.yaml); baseline-конфиги не меняются.

### Сборка

Плагины собираются в рабочем пространстве контейнера `navigation` (`~/ros2_ws`, там смонтирован `navigation/src`):

```bash
docker compose exec navigation bash
source /opt/ros/humble/setup.bash && cd ~/ros2_ws
colcon build --packages-select al_nav2_plugins && source install/setup.bash
```

Тесты пакета (gtest на фикстурах Python-эталона `active_localization` плюс линтеры ament) в отдельном overlay-пространстве `~/dev_ws`, не трогая `~/ros2_ws`: `navigation/src/al_nav2_plugins/tools/build_and_test.sh` (запускать на хосте, нужен docker).

### Запуск режимов

Один и тот же сценарий и одни и те же seed в каждом режиме, каждый прогон стартует с чистого симулятора:

```bash
scripts/run_experiment.sh --mode baseline --runs 3 --scenario <сценарий.yaml> [--seed0 42]
scripts/run_experiment.sh --mode active   --runs 3 --scenario <сценарий.yaml> [--seed0 42]
```

| Режим | Файл параметров Nav2 | Что работает |
|---|---|---|
| `baseline` | `nav2_params_v2.yaml` | Theta* + MPPI как раньше |
| `active` | `nav2_params_active.yaml` (переменная `NAV_PARAMS_ACTIVE`) | `LocalizabilityLayer` + `LocalizationAwarePlanner`, `scorer: "marker_soft_count"` |
| `control` | копия `nav2_params_active.yaml` | те же плагины и тот же MPPI, но без наблюдаемости: `scorer: "null"` в планировщике и в слое, `loc_layer.enabled: false` - так отделяется эффект метода от эффекта смены плагинов и критиков |

Результат каждого прогона: `~/al_runs/<режим>_NN/` (`gt.tum`, `est.tum`, `result.yaml`, ...); дальше `eval/run_evo.sh` и `eval/aggregate.py`. Отдельного флага `--mode control` в `run_experiment.sh` пока нет (ENG добавляет): до тех пор control запускается как `--mode active` с `NAV_PARAMS_ACTIVE`, указывающим на копию файла с перечисленными выше изменениями. [не проверено: после слияния PR ENG свериться с фактическими флагами раннера]

### Основные параметры

Полная таблица - в [README пакета](navigation/src/al_nav2_plugins/README.md#parameters). Те, что чаще всего трогают:

| Параметр | Значение (spec v2) | Смысл |
|---|---|---|
| `loc_layer.c_max` | `40` (**целое**; `40.0` rclcpp отклонит) | максимальная добавка к стоимости клетки, подбирается сухими прогонами |
| `loc_layer.scorer`, `GridBased.scorer` | `marker_soft_count` / `"null"` | наблюдаемость маркеров / выключена (control) |
| `k_a`, `r_min`, `k_r` | `20`, `0.66`, `40` | мягкая видимость маркера по размеру в пикселях и дальности |
| `GridBased.k_yaw_rate` | `2.0` | предел скорости поворота на метр пути в DP курса |
| `GridBased.step` | `0.20` | шаг передискретизации пути; он в ~4 раза реже родного шага Theta* |

### Проверка вживую

Когда Nav2 запущен (в контейнере или с хоста), `tools/check_live.sh` печатает PASS/FAIL: загружены ли классы плагинов, параметры слоя и планировщика (с предупреждением, если они не совпадают со spec v2), стоимость в нескольких клетках и один образец пути с курсом, отличным от касательной:

```bash
navigation/src/al_nav2_plugins/tools/check_live.sh --expect active  --scenario <сценарий.yaml>
navigation/src/al_nav2_plugins/tools/check_live.sh --expect control --goal 1.0,1.0,0.0
```

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
