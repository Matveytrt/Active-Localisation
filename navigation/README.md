# RTAB-Map SLAM, STag маркеры и Nav2

Этот модуль содержит:
- SLAM и локализацию с помощью RTAB-Map (с поддержкой landmarks по STag маркерам)
- детектор STag маркеров (C++)
- навигацию Nav2 по карте RTAB-Map (цель задаётся в RViz)
- инструменты для экспериментов с детекцией маркеров и анализа их результатов.

Сама **симуляция робота Astribot в Gazebo (URDF, миры, объединение лидаров) вынесена в отдельный модуль**:  
[`gazebo`](../gazebo/README.md).

Для полной цепочки *«симуляция → детекция маркеров → SLAM → Nav2»* используйте **оба** модуля (из корня проекта: `docker compose up -d --build`). Контейнеры работают с `network_mode: host` и одинаковым `ROS_DOMAIN_ID`, поэтому видят топики друг друга.

### Структура проекта

- `src/stag_ros2_enhanced` - детектор STag маркеров (`stag_detector_node`, `stag_detection_logger_node`) и генерация маркеров/PDF
- `src/rtabmap_landmarks` - RTAB-Map с лидаром и landmarks, экспорт поз больших маркеров из карты в `landmarks.yaml`, их визуализация
- `src/tf_to_fiducial` - конвертер TF маркеров → `rtabmap_msgs/LandmarkDetections` (C++, медианный фильтр и отсев выбросов)
- `src/nav2_planner_custom` - Nav2 с параметрами под RTAB-Map и деревья поведения; цель задаётся из RViz
- `src/stag_experiment_manager` - оркестратор серий экспериментов с детектором и логгером
- `src/stag_data_analyzer` - анализ и визуализация логов детекции (Plotly)

**`maps/`** - папка для хранения 3D карт RTAB-Map, полученных при построении. Карты сохраняются в формате базы данных RTAB-Map (`.db` файлы) и доступны как volume в Docker контейнере.

Проект использует Docker для изоляции окружения ROS2 Humble и всех зависимостей.

### Работа с Docker контейнером

**Разрешение на использование GUI (для RViz, rqt и других GUI-приложений):**
```bash
xhost +local:docker
```

**Собрать Docker образ:**
```bash
docker compose build
```

**Запустить контейнер:**
```bash
docker compose up -d
```

**Войти в контейнер:**
```bash
docker compose exec navigation bash
```
**Выйти из bash (контейнер продолжит работать):**
```bash
exit
```

**Остановить контейнер:**
```bash
docker compose down
```


### Запуск SLAM, детекции маркеров и Nav2

> **Важно:** ниже приведены команды только для SLAM, landmarks и Nav2.  
> Запуск самой симуляции (робота и мира Gazebo) выполняется из модуля  
> [`gazebo`](../gazebo/README.md) и здесь не описывается.

### Пакет `stag_ros2_enhanced`

#### 1. `stag_detector.launch.py`

Запускает детектор STag маркеров из RGB изображения камеры.

**Что запускает:**
- `stag_detector_node` - узел детекции STag маркеров

**Команда запуска:**
```bash
ros2 launch stag_ros2_enhanced stag_detector.launch.py
```

**Параметры:**
- `config_file` - путь к конфигурационному YAML файлу (по умолчанию: `config/detector_params_HD17.yaml`)

**Публикации:**
- TF трансформации для каждого обнаруженного маркера: `<marker_frame_prefix><id>` (в конфигах HD17/HD15 — `stag_hd17_<id>` / `stag_hd15_<id>`)
- Топики с информацией о детекциях

#### 2. `stag_two_detectors.launch.py`

Запускает два детектора одновременно: для больших маркеров HD17 (namespace `stag_hd17`, `config/detector_params_HD17.yaml`) и маленьких HD15 (namespace `stag_hd15`, `config/detector_params_HD15.yaml`).

```bash
ros2 launch stag_ros2_enhanced stag_two_detectors.launch.py
```


### Пакет `rtabmap_landmarks`

#### `complete_system.launch.py`

**Главный launch файл** - запускает детекцию маркеров и SLAM. Симуляция должна быть уже запущена в контейнере `gazebo`.

**Что запускает:**
1. `stag_two_detectors.launch.py` - детекторы STag маркеров HD15 и HD17
2. `tf_to_fiducial.launch.py` (пакет `tf_to_fiducial`) - большие маркеры HD17 из TF → `LandmarkDetections`
3. `rtabmap_lidar_landmarks.launch.py` - RTAB-Map SLAM с лидаром и landmarks
4. в режиме локализации - `landmarks_yaml_markers_node`, позы маркеров из `config/landmarks.yaml` в RViz

**Параметры** (если не заданы, берутся из `config/rtabmap_lidar_landmarks_params.yaml`):
- `localization` - режим локализации (по умолчанию `false` = картирование)
- `delete_db_on_start` - удалить карту перед картированием (по умолчанию `false`)
- `database_path` - путь к карте (по умолчанию `/home/app/ros2_ws/maps/astribot_sim.db`)

```bash
# Новая карта (старая удаляется)
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false delete_db_on_start:=true
# Продолжить построение той же карты
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false
# Локализация по готовой карте
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=true
```

#### Позы маркеров после картирования

Картирование завершается одним Ctrl+C - RTAB-Map сохраняет оптимизированный граф в базу (дождаться завершения процесса `rtabmap`). Затем позы больших маркеров выгружаются в `config/landmarks.yaml`:

```bash
ros2 run rtabmap_landmarks export_landmarks_from_db
```

#### Другие launch файлы

- `rtabmap_lidar_landmarks.launch.py` - только RTAB-Map
- `landmarks_yaml_markers.launch.py` - только визуализация `config/landmarks.yaml` в RViz
- `tf_to_fiducial.launch.py` из пакета `tf_to_fiducial` - только конвертер TF → `LandmarkDetections`

Подробнее: `src/rtabmap_landmarks/README.md`, `src/tf_to_fiducial/README.md`.

### Пакет `nav2_planner_custom`

Стек Nav2 (MPPI omni, Theta*, behavior tree с восстановлением) поверх карты и локализации RTAB-Map. Нужен запущенный RTAB-Map в режиме локализации (`/map`, TF `map → odom`).

```bash
ros2 launch nav2_planner_custom navigation_launch.py
```

**Параметры:**
- `params_file` - YAML с параметрами Nav2 (по умолчанию `config/nav2_params_v2.yaml` в пакете)
- `use_composition`, `use_respawn`, `log_level`, `namespace`

Цель задаётся в RViz (Fixed Frame `map`) инструментом **2D Goal Pose**. Своего RViz-конфига в пакете нет. Подробности по параметрам и деревьям поведения — в [`src/nav2_planner_custom/README.md`](src/nav2_planner_custom/README.md).

**Примечание:** в сложных ситуациях NAV2 можно помогать, используя управление оператором. Если есть сложности в начале, то нужно немного отъехать со стартовой позиции.

### Пакет `stag_experiment_manager`

Оркестратор автоматических серий экспериментов: запускает детектор с логгером (`stag_detector_with_logging.launch.py`) для каждого типа маркера и попытки и раскладывает результаты по папкам. Конфигурация: `config/experiment_config.yaml`.

```bash
ros2 launch stag_experiment_manager experiment_orchestrator.launch.py
```

Подробнее: `src/stag_experiment_manager/README.md`.

### Пакет `stag_data_analyzer`

Анализ JSONL логов детекции: статистика (FPS, количество детекций, одновременная видимость), heatmap и сравнительные графики (Plotly, экспорт в PNG через kaleido). Конфигурация: `config/analysis_config.yaml`.

```bash
ros2 run stag_data_analyzer analyze_data --config /path/to/config.yaml
```

Подробнее: `src/stag_data_analyzer/README.md`.


## Troubleshooting

### Проблемы с GUI

Если Gazebo или RViz не запускаются:
```bash
xhost +local:docker
```

### Проблемы с джойстиком

Убедитесь, что джойстик подключен и доступен:
```bash
ls /dev/input/js*
```

При отсутствии джойстика можно использовать клавиатурное управление:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Проблемы с картой

Если карта не сохраняется, проверьте права доступа к папке `maps/`:
```bash
docker compose exec navigation ls -la /home/app/ros2_ws/maps
```
