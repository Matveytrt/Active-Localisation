# al_eval — запись траекторий для APE/RPE

Пакет ament_python, узел `traj_recorder`. Пишет две траектории в формате TUM
(`timestamp x y z qx qy qz qw`, секунды с 9 знаками, sim time):

| файл | источник | частота |
|---|---|---|
| `gt.tum`  | топик `gt_topic` (`/odom_gt`, nav_msgs/Odometry) + SE(2)-оффсет `gt_offset_*` | каждое сообщение (planar_move: 50 Гц) |
| `est.tum` | TF `map_frame → base_frame` (`map → base_link` = RTAB-Map `map→odom` ∘ одометрия `odom→base_link`) | таймер `rate_hz` (10 Гц) |
| `meta.yaml` | параметры, время старта/остановки, счётчики пропусков, первая «сырая» поза GT и её `frame_id` | при остановке |

Выход: `<output_dir>/<run_name>/`.

## Запуск

```bash
colcon build --packages-select al_eval && source install/setup.bash
ros2 launch al_eval traj_recorder.launch.py run_name:=baseline_01
# ... прогон ...  Ctrl+C -> файлы закрываются, пишется meta.yaml
```

Аргументы launch (перекрывают `config/traj_recorder.yaml`): `use_sim_time`
(true), `output_dir` (`~/al_runs`), `run_name` (пусто → `run_YYYYmmdd_HHMMSS`),
`rate_hz`, `gt_topic`, `record_gt`, `gt_offset_x`, `gt_offset_y`,
`gt_offset_yaw`, `overwrite` (false: узел откажется писать в каталог, где уже
есть `gt.tum`/`est.tum`), `config`. Числа с плавающей точкой передавайте с
точкой: `gt_offset_x:=-2.0`.

## Как пишется est

* Каждые `1/rate_hz` с: `lookup_transform(map, base_link, Time())` —
  последний доступный трансформ; в файл идёт **stamp самого трансформа**, а не
  время таймера (так ассоциация по времени с GT честная).
* TF недоступен → пропуск, счётчик `est_skipped_tf_unavailable`, первая ошибка
  в `meta.yaml: first_tf_error`.
* Stamp не изменился с прошлого тика (TF не обновился) → пропуск, счётчик
  `est_skipped_duplicate_stamp` (дубликаты времени ломают ассоциацию в evo).

## Кадр GT — НЕ ПРОВЕРЕНО

Неизвестно, в каком кадре поза `/odom_gt` от planar_move: в мировом Gazebo или
относительно точки спавна (робот спавнится в (-2.0, -0.5, 0.16), yaw 0). Кадр
`map` RTAB-Map обычно совпадает с позой робота в момент старта картирования.
Поэтому преобразование GT → `map` — параметр:

```
pose_map = offset ∘ pose_gt,   offset = (gt_offset_x, gt_offset_y, gt_offset_yaw)
x' = ox + cos(oyaw)·x − sin(oyaw)·y;  y' = oy + sin(oyaw)·x + cos(oyaw)·y;  yaw' = oyaw + yaw
```

Как определить оффсет:
1. Записать короткий прогон с оффсетом 0, посмотреть `meta.yaml →
   gt_info.first_pose_raw` (и `frame_id`) сразу после спавна: ≈(0, 0) —
   поза относительно спавна, ≈(-2.0, -0.5) — мировая.
2. Сравнить с первой строкой `est.tum` (поза в `map`).
3. Либо `eval/run_evo.sh <run> --align posyaw` — печатает оценку оффсета
   (`inverse (gt_offset_x/y/yaw candidate)`). Подставить её и перезаписать.

Если GT берётся из TUM-файла плагина pose_saver — `record_gt:=false`, а файл
положить в каталог прогона как `gt.tum` (или `run_evo.sh --gt <имя>`).
Внимание: оффсет к такому файлу узел не применяет.

## Тесты

```bash
python3 -m pytest navigation/src/al_eval/test   # чистый Python, ROS не нужен
colcon test --packages-select al_eval
```

Оценка (evo, агрегирование) — см. `eval/README.md`.
