# Итерация 008 — общий runner baseline/active/control

09.11.2026, по прямой просьбе пользователя. Статус: принято как общий runner
и один полный paired pilot baseline/active/control, без статистики преимущества.
Стартовый commit 4450b79366e0ce6eb7837f898e5d39068379cf87, обе копии чистые.

Отдельный privacy commit 3c37481 скрыл в текущих tracked документах детали
подключения, hostname и стороннего проекта, добавил правило не публиковать их.
Он не переписывает старую Git-историю; SSH connection config остаётся вне Git.

## Критерии, заданные до прогонов

- Один runner, lock и отказ от перезаписи. Только собственные контейнеры.
- В каждой серии одинаковые pristine DB/landmarks/scenario/start и noise/lidar seeds.
  Полный новый simulator/ROS/odom/RNG, отдельная DB-копия и собственные run/config/logs.
  MPPI RNG не управляется этим adapter; не заявлять bitwise deterministic replay.
- Исходные алгоритмы сохраняются. Общие resource/velocity overrides из 007.
  Control отличается от active ровно prescribed scorer=null в planner/layer,
  loc_layer.enabled=false. В обоих новый exported Isaac landmarks YAML.
- Реальные runtime params/logs подтверждают загрузку planner/layer и 5 markers.
- Исходный shelves_south_wall, минимум один полный run каждого режима, оба goal
  SUCCEEDED, estimated XY ≤.15 m/yaw ≤.10 rad, final GT XY ≤.30 m.
- В активном режиме фактически отличается route (≥.10 m средний y) или robot yaw
  (≥.20 rad средняя абсолютная разница/смещение относительно control) на длинном
  южном участке; сохранить исходные /plan и trajectory, не выводить эффект лишь из YAML.
- GT/est/map_odom/odom TUM, landmarks, clocks/RTF, feedback/recoveries, seeds,
  hashes/versions/metadata и result сохранены; stamps не идут назад, finite poses.
- Cancel/error/cleanup не выдаются за success. Pristine SHA после runs неизменен.
  Длительная статистика, APE/RPE и доказательство выигрыша — этап 009.

## Промежуточные проверки

11 adapter/config/preflight tests passed. Pilot `pilot-008-a`: обе baseline цели
succeeded (GT XY .01787/.07287 m, estimated XY .01577/.07339 m,
yaw .03821/.04739 rad, navigation RTF .95685), но общий запуск **не принят**:
SIGTERM не прошёл через `ros2 run` к дочерним lidar/STag C++ processes (exit137,
OOM=false), а noise повторно вызывал shutdown (exit1). Simulator/SLAM/Nav2/recorder
завершились штатно. Артефакты failed batch сохранены, active/control не запускались.

Исправление adapter: direct exec исходных бинарников lidar/STag, SIGINT для ROS
процессов, rclpy.try_shutdown и обработка штатного завершения noise. Сам ScenarioRunner
теперь живёт в отдельном собственном контейнере: timeout/cancel host runner не оставит
docker-exec child, способного отправить цель в следующий эпизод. Добавлена запись
clock/wall. Исходные алгоритмы и их C++/Python packages не изменены.

## Реализация

`isaac/experiment_config.py` читает исходные nav2_params_v2/active и накладывает
общий профиль скоростей .6/.6/.8, новый путь exported landmarks и prescribed
три control deltas. Bounded SLAM вынесен туда же: MemoryThr=200,
map_filter_radius=.15, map_filter_angle=10°, map_cache_loaded_on_init=false.
Ни world source poses, ни GT в localization/planning не поступают.

`run_experiment.py` — Linux runner на удалённом ПК. flock исключает второй запуск;
проверяются pristine SHA, seed, место, занятые собственные контейнеры и отсутствие
каталога результатов. Для каждого режима — новая writable DB, полный новый
Isaac/noise/STag/SLAM/Nav2/recorder, одинаковый fixed spawn [-2,-.5,.16,yaw0],
один /initialpose. Ручных Twist во время сценария нет; cleanup только своих процессов.

`episode.py` использует исходный ScenarioRunner и NavigateToPose/result.yaml;
добавляет lifecycle ACTIVE readiness, проверку runtime параметров/seeds, GT-only
evaluation, запись raw noisy odom/plans/feedback/clock-wall. Исходный TrajRecorder
пишет gt.tum, est.tum, map_odom.tum, landmarks.csv, meta.yaml. Симулятор сообщает
lidar seeds front42/back43; noise seed42 проверяется ROS service.
MPPI RNG не управляется. Входы совпадают, побитовый replay движения не обещан.

## Итоговый pilot `pilot-008-b`

Исходный `shelves_south_wall`, цели [-2,-1.4,yaw0] и [2.6,-1.4,yaw0],
исходный timeout180s/settle2s. Все 6 goals SUCCEEDED, recoveries=0.

| Режим | GT XY целей, cm | Estimated XY, cm | Max final yaw, rad | Goal wall sum, s | Navigation RTF | Landmark messages |
|---|---|---|---|---|---|---|
| baseline | 2.28 / 6.66 | 1.87 / 6.57 | .04637 | 12.446 | .9641 | 0 |
| active | 8.10 / 1.83 | 7.39 / 1.88 | .03805 | 17.985 | .9692 | 30 |
| control | 6.53 / 5.35 | 6.45 / 5.50 | .04697 | 17.245 | .9674 | 0 |

Все 30 сообщений active содержали реальные accepted HD17 IDs1/2; marker pose
не подменён симулятором. Нулевые streams baseline/control сохранены как есть.
Active runtime подтвердил LocalizationAwarePlanner/marker_soft_count, enabled
LocalizabilityLayer и по 5 markers; control — тот же planner/scorer=null,
layer scorer=null/enabled=false. Active layer реально вычислял 187x172/188x173
cells за .948–1.244s (logs), а не только присутствовал в YAML.

`analyze_experiment.py`: реальные GT trajectories на goal2, общие 10cm bins
x∈[-1,1.5): разница среднего y **.443895m**, средняя абсолютная разница heading
**.951849rad** (54.54°). Оба предзаданных порога .10m/.20rad пройдены.
PNG просмотрен: active смещается к маркерам и поворачивает корпус, control держит
южный коридор и примерно нулевой yaw. Raw /plan сохранены: 13/17/18 plans.
Это подтверждение исполнения active objective, а не оценка выигрыша по APE.

Pristine DB SHA до/после каждого режима и на ноутбуке:
`70a236237a60e4c4294d31ab308f449bdaa1e5be5dab5399742701a8ed6397fe`.
Начальная DB каждого режима имеет тот же SHA. Landmarks SHA
`9a6f762a31ef4df524dfff2b2401a8d66f2ff0180e0fc498d98c784c30830f11`.
Scenario SHA `9df9cc2461edc3ccf7fd399fdc3535989950a8de186b15db487bc1fee5b52403`.
Source hashes/config/image IDs/versions — manifest/config/containers/versions.
Runs выполнялись с uncommitted adapter поверх 3c37481: manifest фиксирует hashes,
это не выдаётся за чистый commit replay. После runs уточнён только shell разбор
leading-zero seeds, проверенный отдельными 9 CLI tests; seed42 не изменён.

TUM GT/est/map_odom/noisy: 1010/197/198/850 samples baseline,
1349/259/260/1075 active, 1246/241/242/1049 control. Все finite, unit quaternions,
строго возрастающие timestamps. Clock/wall монотонны (178/234/213 samples).
Начальная TF недоступность recorder сохранена как counters, без идеальных поз.
TF publishers — noise, RTAB и два STag; отдельные map→odom/odom→base сохранены.

Каждый эпизод: все 10 собственных контейнеров exit0/OOM=false, включая
simulator/SLAM/Nav2/STag/lidar/noise/recorder/episode. Сторонние контейнеры не менялись.
После серии работает только свой idle navigation service. Full bridge RTF
.9700/.9735/.9739; startup readiness 141.4/139.2/137.3s.

## Проверки и артефакты

35 C++ passed; 190 Python passed/1 skipped (включая 11 новых config/preflight),
после уточнения shell parser ещё 9 seed CLI passed. Live concurrent runner
отвергнут flock до создания каталога/процессов. Wrong-map, existing-result,
busy-simulator/path traversal negative controls входят в 11 tests.
Shell syntax/Compose/Python compilation прошли. Реальный timeout активной Nav2
цели отдельно не инжектировали; это ограничение failure coverage.

Remote/local `isaac/runtime/navigation/experiments/pilot-008-b/`: manifest.json,
analysis.json, comparison.png, versions.log; по режиму result/acceptance/meta,
четыре TUM, plans/feedback/clock-wall JSONL, landmarks.csv, configs/parameters,
container/bridge results/logs. Failed `pilot-008-a` сохранён отдельно.
`isaac/runtime/evidence/*008*`: runner/analysis/lock/tests logs и архив зеркала.
Writable episode DB под runtime/navigation/maps/episodes/ остаются на удалённом ПК;
на ноутбуке canonical pristine DB и все небольшие результаты. Runtime вне Git.

Решение: 008 принята в указанных пределах. Следующий этап 009 — парные серии
с разными seeds и сбалансированным порядком, formal timestamp association APE/RPE,
success/time/recoveries/detections/RTF и интервалы неопределённости. Один pilot
не доказывает статистическое преимущество, длительную стабильность или колесную физику.
