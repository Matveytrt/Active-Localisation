# tf_to_fiducial

C++ нода: превращает TF-фреймы больших STag-маркеров (в симуляции HD17, `stag_hd17_<id>`) в `rtabmap_msgs/LandmarkDetections` — так маркеры становятся ландмарками графа RTAB-Map.

## Запуск

```bash
ros2 launch tf_to_fiducial tf_to_fiducial.launch.py
# входит в complete_system.launch.py; аргумент localization передаётся туда как operation_mode
```

## Как работает

Таймер `update_rate` (10 Гц) перебирает id из диапазона/allowlist, ищет TF `base_link → stag_hd17_<id>`, копит скользящее окно поз (медиана по `mapping_pose_sample_count`/`localization_pose_sample_count` сэмплов) и публикует в `/landmark_detections`. Выбросы (`outlier_gate_m`) и устаревшие TF (`max_tf_age_sec`) отбрасываются.

## Конфиг `config/tf_to_fiducial_params.yaml` — важное

| Параметр | Значение | Смысл |
|---|---|---|
| `allowed_marker_ids` | `[1, 2, 3, 4, 5]` | Какие большие маркеры идут в RTAB-Map. Если строку закомментировать, используется весь диапазон `marker_id_range_*` |
| `marker_frame_prefix` | `stag_hd17_` | Префикс TF-фреймов детектора |
| `marker_size` | `0.08` | Сторона чёрного квадрата маркера HD17, м |
| `operation_mode` | `mapping` | `mapping` / `localization`; обычно задаётся launch-аргументом |
| `max_publish_distance_m` | 9.0 | В localization-режиме маркеры дальше не публикуются |
| `mapping_pose_sample_count` | 3 | Окно медианы (сэмплов при 10 Гц) |
| `use_sim_time` | `true` | Время из `/clock` симуляции |

RTAB-Map принимает ландмарки только с id > 0, поэтому большой маркер HD17 с id 0 в граф не попадает.
