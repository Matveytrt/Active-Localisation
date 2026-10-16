# nav2_planner_custom

Nav2 для симуляции Astribot поверх RTAB-Map: launch навигации, параметры Nav2 и behavior trees. Перенесено из исходного репозитория навигации без вспомогательных серверов (подъезд и выравнивание по маркерам, снимки, py_trees, keepout): цель задаётся в RViz, робот едет.

## Запуск

Нужны запущенная симуляция (модуль `gazebo`) и RTAB-Map в режиме локализации по готовой карте (`rtabmap_landmarks`, `complete_system.launch.py localization:=true`) — он публикует `/map` и TF `map → odom`.

```bash
ros2 launch nav2_planner_custom navigation_launch.py
# аргументы: params_file, use_composition, use_respawn, log_level, namespace
```

Цель: в RViz (Fixed Frame `map`) инструмент **2D Goal Pose** (топик `/goal_pose`) — bt_navigator принимает его как цель `NavigateToPose`. Для отображения удобно добавить `Map` (`/map`), `Map` (`/global_costmap/costmap`, `/local_costmap/costmap`), `Path` (`/plan`), `RobotModel`; панель `Navigation 2` (из `nav2_rviz_plugins`) показывает статус цели и позволяет её отменить.

## Конфиг `config/nav2_params_v2.yaml` — ключевое

| Что | Значение | Комментарий |
|---|---|---|
| Контроллер | **MPPI** (`nav2_mppi_controller`), 10 Гц | Омни-кинематика (`motion_model: Omni`) — как у `gazebo_ros_planar_move`; `batch_size: 1000`, `time_steps: 24` — главные рычаги CPU |
| Планировщик | Theta* (`GridBased`) | `w_traversal_cost: 0.5` |
| Скорости | 1.25 м/с, 1.5 рад/с (velocity_smoother); MPPI до 1.0 м/с | `cmd_vel` контроллера → `cmd_vel_raw` → velocity_smoother → `/cmd_vel` |
| Footprint | прямоугольник 0.65 × 0.75 м | Одинаковый в обеих костмапах |
| Локальная костмапа | 3×3 м, 0.05 м, 5 Гц | Источник — `/lidar/points_joined` (lidar_stitching) |
| Глобальная костмапа | `static_layer` (`/map` от RTAB-Map) + `obstacle_layer` ДО `denoise_layer` + `inflation_layer` | Порядок plugins критичен: denoise фильтрует шум obstacle_layer |
| Высоты препятствий | от 0.05 до 1.75 м в `global_frame` костмапы (`odom` / `map`) | Nav2 сравнивает высоту не в `base_link`, а в глобальном кадре костмапы. В симуляции `base_link` на z = 0.16, пол в `odom`/`map` на z ≈ 0.0; порог ниже нуля пропускает пол в костмапы |
| `use_sim_time` | `true` во всех узлах | Задаётся только в YAML, launch его не переопределяет |
| `use_realtime_priority` | `false` | `true` требует SCHED_FIFO, а в контейнере он запрещён |
| `bond_timeout` | 0.0 | Защита от ложного «SERVER IS DOWN» при перегрузке CPU: проверка bond выключена |
| `transform_tolerance` | 30.0 | Защита от отказов из-за запаздывающих TF при перегрузке CPU |

## Behavior tree

`behavior_trees/navigate_to_pose_w_recovery_v3.xml` — дефолт (`bt_navigator.default_nav_to_pose_bt_xml`). Робастное восстановление при потере локализации: лестница `RoundRobin` (клир вокруг робота + выдержка → `Spin` 30° → глубокий клир), у планировщика нет inline-recovery. Подробный разбор — в шапке XML.

`navigate_to_pose_w_fast_failure.xml` (v1) и `navigate_to_pose_w_fast_failure_v2.xml` — альтернативы: в YAML поменять имя файла в `default_nav_to_pose_bt_xml` — launch берёт оттуда только имя и подставляет путь к установленному `behavior_trees/`.

Отдельного ограничения времени восстановления нет: цель из RViz обрывается деревом после `number_of_retries="3"`.
