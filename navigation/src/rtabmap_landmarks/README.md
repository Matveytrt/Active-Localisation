# rtabmap_landmarks

SLAM для симуляции Astribot: launch-файлы RTAB-Map (лидар + STag-ландмарки), экспорт поз больших маркеров из карты в `landmarks.yaml` и их визуализация в RViz. Чистый `ament_python`.

Большие маркеры в симуляции — HD17, они идут в граф RTAB-Map как ландмарки (через пакет `tf_to_fiducial`). Малые HD15 детектируются, но в карту не заносятся.

## Launch-файлы

| Launch | Что делает |
|---|---|
| `complete_system.launch.py` | **Главный**: STag-детекторы HD15/HD17 + `tf_to_fiducial` + RTAB-Map; в режиме локализации ещё и визуализация маркеров. Режим (карта/локализация) — аргумент `localization`, дефолт из конфига |
| `rtabmap_lidar_landmarks.launch.py` | Только RTAB-Map (включается из complete_system) |
| `landmarks_yaml_markers.launch.py` | Только визуализация маркеров из `landmarks.yaml` в RViz |

Симуляция должна быть уже запущена в контейнере `gazebo`.

```bash
# Новая карта (старая удаляется)
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false delete_db_on_start:=true
# Продолжить картирование той же карты
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false
# Локализация по готовой карте
ros2 launch rtabmap_landmarks complete_system.launch.py localization:=true
```

Аргументы `complete_system.launch.py`: `localization`, `delete_db_on_start`, `database_path` (пусто = из конфига).

## Главный конфиг: `config/rtabmap_lidar_landmarks_params.yaml`

Секция `launch_config` — то, что переключается чаще всего:

| Параметр | Значение | Смысл |
|---|---|---|
| `localization` | `false` | Режим по умолчанию |
| `delete_db_on_start` | `false` | Удалять карту при старте картирования |
| `database_path` | `/home/app/ros2_ws/maps/astribot_sim.db` | Карта (папка `navigation/maps` на хосте) |
| `mapping_params` / `localization_params` | | Параметры RTAB-Map, зависящие от режима (`Mem/IncrementalMemory`, `RGBD/CreateOccupancyGrid`, `Grid/3D` и др.) |

Там же топики: `lidar_topic: /lidar/points_joined`, `odom_topic: /odom`, `landmark_detections_topic: /landmark_detections`. Ниже в файле — параметры алгоритма RTAB-Map (ICP, `Rtabmap/DetectionRate: 5`, доверие к ландмаркам, occupancy grid) — менять осознанно. `use_sim_time: true` задаётся только здесь.

## Маркеры: `config/landmarks.yaml`

Позы больших маркеров в кадре `map` текущей карты. Заполняется после картирования экспортом из базы RTAB-Map:

```bash
ros2 run rtabmap_landmarks export_landmarks_from_db \
    --db /home/app/ros2_ws/maps/astribot_sim.db \
    --out /home/app/ros2_ws/src/rtabmap_landmarks/config/landmarks.yaml
```

Позы берутся из оптимизированного графа, который RTAB-Map сохраняет в базу при штатном завершении картирования: Ctrl+C один раз и дождаться, пока процесс `rtabmap` завершится (повторный Ctrl+C может прервать запись базы). Если графа нет, скрипт предупреждает и считает позы по сырой одометрии узлов — это менее точно.

Формат:

```yaml
frame_id: map
big_marker_family: HD17
big_markers:
- id: 1
  order: 1
  position: {x: 0.11, y: 0.93, z: 1.23}
  orientation: {x: 0.0, y: 0.0, z: -0.69, w: 0.73}
```

Файл лежит в `src` (смонтирован в контейнер), поэтому правка руками и повторный экспорт видны без пересборки; узел визуализации перечитывает его при изменении.

## Прочие конфиги

- `config/landmarks_yaml_markers_params.yaml` — параметры узла визуализации (топик `/visualization_marker_array`).

## После построения карты

```bash
rtabmap-databaseViewer /home/app/ros2_ws/maps/astribot_sim.db
# вкладка Detect more loop closures → дождаться → сохранить при выходе
```
