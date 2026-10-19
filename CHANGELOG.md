# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/).

## [Unreleased]

### Repository polish (T-012c)

- Single English root `README.md` (overview, testbed, method, build, map, run, evaluate, report, limitations).
- `scripts/get_map.sh`: map database download from a GitHub Release with SHA-256 verification (placeholders until the asset is uploaded).
- `LICENSE` placeholder (TODO: choose a license), extended `.gitignore`, executable bits on scripts.

Первая версия репозитория `active-localization`: два независимых Docker-модуля, которые общаются только через ROS 2 топики.

- `gazebo` — симуляция робота Astribot в Gazebo 11: мир с маркерами STag HD15/HD17, омни-база через `gazebo_ros_planar_move`, объединение облаков переднего и заднего лидаров (`lidar_stitching`).
- `navigation` — детектор STag (C++), RTAB-Map по лидару с большими маркерами HD17 как ландмарками (`rtabmap_landmarks`, `tf_to_fiducial`), экспорт поз маркеров из карты в `landmarks.yaml` и их отображение в RViz, Nav2 (MPPI, Theta*) с целью из RViz (`nav2_planner_custom`), инструменты экспериментов и анализа детекций.
