# Active Localization

Симуляция робота Astribot, SLAM по лидару и STag маркерам (RTAB-Map) и навигация (Nav2).

> **Active localization experiment:** method, baseline/active/control runs, metrics and the
> report are described in [Reproduce the experiment](#reproduce-the-experiment) (English).

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
| `loc_layer.c_max` | `150` (**целое**; `150.0` rclcpp отклонит) | максимальная добавка к стоимости клетки (DIR 18.10, по офлайн-прогнозу T-011b) |
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


## Reproduce the experiment

Skeleton (T-012a). Commands marked **TODO(ENG)** are not final yet; everything
else is taken from the existing READMEs (`scripts/README.md`, `eval/README.md`).

The experiment compares three modes on the same scenario, start pose, map and
odometry-noise seeds (≥ 3 runs each):

| Mode | Nav2 stack |
|---|---|
| `baseline` | stock Nav2 (`nav2_params_v2.yaml`, BT v3), unchanged |
| `active` | same BT + `al_nav2_plugins` (`LocalizabilityLayer` route lever, `LocalizationAwarePlanner` yaw lever) via a separate active params file |
| `control` | active stack with the `null` scorer (layer cost 0, headings = direction of travel); must match `baseline` |

### 1. Prerequisites

- Docker with Docker Compose >= 2.20.3, X server access (`xhost +local:docker`), see [Требования](#требования).
- GPU rendering of the camera is strongly recommended (on CPU the camera runs far below 30 Hz); WSL2 notes: TODO(ENG).
- Host Python >= 3.10 with `evo` 1.38 (`pip install evo`, best in a venv; needs matplotlib >= 3.6), see `eval/README.md`.
- For the report: `latexmk` + `pdflatex` (TeX Live), `pdfinfo` (poppler-utils).

### 2. Build

```bash
docker compose up -d --build
docker compose exec navigation bash -c \
  'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select al_eval al_odom_noise'
# TODO(ENG): build al_nav2_plugins (C++) and active_localization in the image / workspace
```

### 3. Map

The RTAB-Map database is not in git (`navigation/maps/*.db`, ≈150 MB). Build it
once with the mapping steps of the [typical scenario](#типовой-сценарий-симуляция--slam--навигация)
(`complete_system.launch.py localization:=false delete_db_on_start:=true`, drive
around, `export_landmarks_from_db` → `landmarks.yaml`) and save it as
`navigation/maps/astribot_sim.db`.
TODO(ENG): exact mapping route/script, or a download link for the map used in the report.
Each run starts from a pristine copy of this DB (localization mode writes into it).

### 4. Run baseline / active / control

```bash
S=navigation/src/al_eval/config/scenarios/example.yaml   # TODO(ENG): final scenario file
scripts/run_experiment.sh --mode baseline --runs 3 --scenario $S --seed0 42
NAV_LAUNCH_ACTIVE='TODO(ENG): ros2 launch ... params_file:=<nav2_params_active.yaml>' \
  scripts/run_experiment.sh --mode active --runs 3 --scenario $S --seed0 42
# TODO(ENG, T-009d): with the C++ plugins the active mode needs no ACTIVE_LAUNCH and uses
#            /navigate_to_pose (runner defaults are still ACTIVE_LAUNCH=active_localization.launch.py,
#            ACTION_ACTIVE=/al/navigate_to_pose; note ACTIVE_LAUNCH='' is replaced by the default,
#            because the runner uses ${VAR:=default})
# TODO(ENG): control mode = active params with scorer null (the runner has no --mode control yet;
#            e.g. NAV_LAUNCH_ACTIVE=<null-scorer params> ... --out ~/al_runs_control)
```

Add `--dry-run` to print every command without running anything. Run `i` uses
seed `seed0 + i - 1` in every mode. Options and exit codes: `scripts/README.md`.

### 5. Evaluate

```bash
# APE/RPE (translation + rotation), no alignment, cropped to the goal window
for r in ~/al_runs/{baseline,active,control}_0{1,2,3}; do eval/run_evo.sh "$r"; done
python3 eval/summarize.py ~/al_runs --out eval_out/          # tables (+ summary_table.tex)
python3 eval/plots.py ~/al_runs --out eval_out/plots --score-map   # figures
```

Details (evaluation window, metrics, paired view, sanity flag): `eval/README.md`.
`eval/aggregate.py` (APE/RPE only, two labels) still works on the same `evo_none/`.

### 6. Where the results go

| Path | Content |
|---|---|
| `~/al_runs/<mode>_<NN>/` | `gt.tum`, `est.tum`, `meta.yaml`, `result.yaml`, `scenario.yaml`, `run.yaml`, `logs/` (one dir per run) |
| `~/al_runs/<mode>_<NN>/` (eval) | `*_eval.tum`, `landmarks_eval.csv`, `eval_window.yaml` (`eval/crop_run.py`, called by `run_evo.sh`) |
| `~/al_runs/<mode>_<NN>/evo_none/` | evo results `ape.zip`, `rpe.zip`, `ape_rot.zip`, `rpe_rot.zip`, plots, `commands.txt`, `eval_window.yaml` |
| `results/` | `results.csv`, `summary.md`, `ape_rmse_boxplot.png` (`eval/aggregate.py`) |
| `eval_out/` | `summary.{csv,md}`, `summary_modes.csv`, `summary_paired.csv`, `summary_table.tex` (read by the report), `plots/` (copy the PDFs to `report/figures/`) |

### 7. Build the report

```bash
report/build.sh        # -> report/report.pdf, fails if it is longer than 2 pages
```

The results table is taken from `eval_out/summary_table.tex` when it exists;
expected figure names are listed in `report/figures/README.md`.

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
