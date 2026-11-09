# Итерация 007 — собственная карта RTAB-Map и baseline

09.11.2026, продолжение по разовому запуску 11:23:48 МСК. Статус: карта и один baseline pilot приняты в указанных границах.
Стартовый commit c4f62efd86dd4f0be982efb69553ea415fff26cb, обе копии чистые.
Исходный world/STag этап 006 принят с ограничениями боковых PnP и raw false IDs.

План: отдельная новая DB из настоящих lidar/odom/landmarks, исходный fiducial
allowlist 1–5/age/outlier gates, экспорт optimized landmarks, новый запуск
локализации из отдельной копии DB и проверка baseline Nav2 через action API.
Тяжёлая работа только на сервере; командные контейнеры не менять.

## Критерии до живой приёмки

- Ненулевые HD17 IDs из исходного мира видны в реальном TF и /landmark_detections.
  В landmark stream только IDs 1–5; ID0/116/202 не допускаются; старые TF
  при взгляде на пустую стену прекращают свежие detections в пределах age gate.
- Карта только из сенсоров и noisy odom, без GT/source marker poses на вход SLAM.
  Mapping-driver использует map→base (либо noisy odom до готовности map TF).
  GT используется только для записи и оценки.
- Новая непустая DB, ≥50 pose nodes, ≥2 ненулевых landmark в optimized graph;
  occupancy grid с free/occupied cells. Экспорт landmarks только штатным exporter
  из новой DB после чистого shutdown; согласованность масштаба/поз с исходником
  оценивается отдельно, ориентир median XY error <.20 m после начального выравнивания.
- Свежая локализация из отдельной копии DB; исходная mapping DB не меняется.
  map→odom публикует только RTAB-Map, odom→base_link — только noise.
- Baseline: ≥3 заданных свободных целей, реальный NavigateToPose action success
  для каждой; final error в map ≤.15 m/.10 rad, фактическая GT XY error ≤.25 m
  относительно map goal с учётом измеренного начального map/world alignment.
  Нет ручных команд движения во время Nav2; watchdog/collision guard активны.
- Частоты/RTF, ресурсная нагрузка, trajectory и ошибки сохраняются. Длительная
  статистика и active/control не принимаются этим этапом.

## Интеграция

Собственные fiducial/slam/baseline сервисы. Алгоритмические параметры RTAB-Map
и fiducial читаются из исходных YAML; меняются режим и путь своей базы.
В baseline только общий override скоростей MPPI/smoother до физических лимитов
Isaac .6/.6/.8, иначе исходные 1.0/1.25 противоречат clamp адаптера.
Это должно одинаково применяться к будущим active/control для честного сравнения.

## Отладка окружения и ограничения памяти

Первый запуск RTAB-Map не загрузил libdiagnostic_updater.so: pinned desktop image
содержал diagnostic_updater 4.0.6 (header-only), а установленный бинарник
rtabmap_slam 0.23.13 слинкован с этой библиотекой. Отдельный Dockerfile RUN
обновляет dependency до проверенной 4.0.7. Обновлены только собственные ROS-контейнеры;
runtime Isaac и контейнеры команды не менялись. Evidence build-navigation-007.log,
versions-007.txt. Source algorithms сохранены; алгоритмические defaults Humble не мигрировали.

Первый полный mapping с неограниченной WM дошёл до всех пяти ID, но около
912 узлов память SLAM выросла до снимка 4.315 GiB. Host 16 GiB и swap были
исчерпаны, Isaac получил OOM kill (exit=1, OOMKilled=true). Route timeout
на возврате домой — следствие остановившегося clock, не успешная приёмка.
Сохранены mapping-oom-007.json/.csv, slam-first-007.log и отдельная failed DB
isaac-20261109-1123.db; она не является принятой картой.

Введён явный общий ресурсный профиль Isaac, отличный от исходных unlimited defaults:

- Rtabmap/MemoryThr='200': WM ограничена, старые узлы переходят в LTM/DB;
- map_filter_radius=.15 m, map_filter_angle=10°: фильтр поз для сборки occupancy cache;
- map_cache_loaded_on_init=false: без полного предварительного разворачивания local grids.

Это реальные изменения параметров исполнения, их нельзя скрывать при сравнении.
Профиль применяется и mapping/localization, должен быть общим для baseline/active/control.
При сравнении именно симуляторов аналогичные условия следует обеспечить и Gazebo.
Официальные определения: [MemoryThr](https://introlab.github.io/rtabmap/api/latest/classrtabmap_1_1Parameters.html),
[MapsManager 0.23.13](https://raw.githubusercontent.com/introlab/rtabmap_ros/0.23.13/rtabmap_util/src/MapsManager.cpp).

Повторный route полностью passed. Пиковая cgroup memory SLAM 1,938,497,536 bytes
(1.81 GiB); перед завершением снимок около 1.5 GiB. База, raw данные и constraints
не удалялись ради лимита. Mapping runtime: 197.116677 s sim /199.825659 s wall,
status=finished, exit=0/OOM=false.

## Карта и native global consolidation

Mapping-driver использовал только оценку map→base (до её готовности — noisy odom),
двигал через /cmd_vel по зафиксированному route и вернулся к старту. GT записывался
отдельно для оценки. Ненулевой HD17 ID1: реальная /landmark_detections в base_link,
ошибка world-position 9.27 mm относительно исходного лица маркера.
В stream были только IDs [1,2,3,4,5], включая все пять. При повороте от маркеров
после 1 s выдержки следующий 1 s не дал новых landmark messages. Исходный age
parameter .3 s и median/outlier gates сохранены; точная временная граница .3 s
не измерялась отдельным sweep-тестом.

После clean shutdown сохранён online DB isaac-20261109-bounded.db, 947 узлов,
219 landmark links. Nav2 map_saver записал реальный occupancy grid 187×172, .05 m/cell.
При bounded WM штатный Admin.opt_poses в online DB содержал лишь один последний
landmark. Поэтому прямой exporter/evaluator правильно не прошёл критерий ≥2;
fallback на raw-odometry links не принимался.

Добавлена отдельная offline-финализация **копии** DB при остановленном симуляторе:
маленький C++ adapter вызывает установленный RTAB-Map getGraph(optimized=true,
global=true), включая LTM, затем сохраняет результат штатным DBDriver.saveOptimizedPoses.
Источник — только реальные записанные граф/constraints. Ни source world, ни GT
программа не читает. Получено 947 узлов, 5 landmarks, 2807 native graph links.
Именно после этого неизменённый exporter проекта выгрузил все пять из optimized_graph.

audit_map_data.py побайтово сравнил таблицы sensor Data, constraints Link и сырые
Node poses online/global DB: hashes совпали. Менялся optimized cache, а не наблюдения
и не подставлялись идеальные позы. Helper finalize_mapping.sh отдельно повторён
на replay-копии, экспорт/eval прошли. Negative control на пустом графе возвращает
ошибку и не создаёт успешный landmark YAML; pipeline не маскирует failed optimization.

Принятая pristine DB: isaac/runtime/navigation/maps/isaac-20261109-global.db.
SHA-256: 70a236237a60e4c4294d31ab308f449bdaa1e5be5dab5399742701a8ed6397fe.
Для baseline создана отдельная isaac-20261109-baseline.db; pristine hash после прогона совпал.

| HD17 ID | XY error экспортированной optimized pose до source face, m |
|---|---|
| 1 | .00215 |
| 2 | .00918 |
| 3 | .01923 |
| 4 | .01740 |
| 5 | .01849 |

Median .01740 m, все пять <.020 m; первоначальный допуск median <.20 m выполнен.
Начальный map/world alignment получен только для оценки, не отправлялся в SLAM.
Диагностика sampled mapping trajectory (837 samples): XY RMSE SLAM .05925 m,
noisy odom .32513 m. Это сравнение с latest TF/odom без точной temporal interpolation,
не формальные APE/RPE и не парный статистический эксперимент.
Карта/траектория: mapping-007/map-and-trajectory.png; JSON рядом.

## Свежая локализация и реальный baseline Nav2

Симулятор полностью перезапущен, noise/RNG и оба детектора пересозданы. RTAB-Map
localization читает отдельную копию global DB. Единственный initialization seed:
известный scenario spawn [-2,-.5,.16,yaw=0] через /initialpose, без live GT correction.
Первый probe отправил goal после action discovery до lifecycle activation и получил
rejected; движения не было. Исправлено ожидание active для controller/planner/
bt_navigator/velocity_smoother. Этот failed probe сохранён как lifecycle-race evidence;
принятый probe стартовал после активации в той же свежей, ещё не двигавшейся сессии.

| Goal map [x,y,yaw] | Status | Wall s | Estimated XY m | Estimated yaw rad | GT XY m |
|---|---|---|---|---|---|
| [-1,-.5,0] | SUCCEEDED (4) | 3.02 | .06544 | .03598 | .06244 |
| [1.9,-.5,pi/2] | SUCCEEDED (4) | 7.62 | .01752 | .04319 | .01043 |
| [2.65,2.4,pi/2] | SUCCEEDED (4) | 6.42 | .08965 | .03152 | .08927 |

Все заранее заданные допуски .15 m/.10 rad estimate и .25 m GT выполнены.
Во время этих целей probe не публиковал ручные Twist. Отдельный read-only graph
audit подтвердил /cmd_vel publishers только Nav2 behavior_server/velocity_smoother,
/map только RTAB-Map, /odom только noise; map→odom и odom→base_link, без ideal TF.
Pristine DB не изменилась. Ground truth использовалась только для оценки результата.

Localization memory peak 1,110,212,608 bytes (1.03 GiB), Nav2 175,366,144 bytes
(167 MiB). Симулятор этой сессии 361.750019 s sim /367.496340 s wall, exit=0/OOM=false;
RTAB-Map завершился exit=0/OOM=false. Первая остановка Nav2 через Docker SIGTERM
потребовала SIGKILL (exit137, OOM=false); для launch добавлен stop_signal SIGINT.
Отдельная проверка start/stop с SIGINT завершилась exit0/OOM=false, без симулятора
и без новых целей. Failed shutdown evidence сохранён, не выдаётся за clean exit.

## Проверки, артефакты и границы

35 C++ tests: 0 errors/failures; 179 Python passed, 1 skipped после обновления
образа. C++ finalize tool собран и проверен на настоящей DB; replay и отрицательный
контроль прошли. Shell syntax и Python compilation проверены.

Основные артефакты под isaac/runtime/navigation/: mapping-007/{result.json,
trajectory.csv,map-and-trajectory.png,trajectory-evaluation.json}, baseline-007/
{result.json,trajectory.csv,graph-audit.json}, maps/*.db/*.yaml/*.pgm/*.evaluation.json.
Runtime/evidence/: mapping-bounded-007.log, global-finalize-007.log,
map-data-audit-007.log, baseline-final-007.log, graph-audit-007.log,
mapping-runtime-007.json, baseline-runtime-007.json, memory peaks и versions-007.txt.
Исходники и отчёты — Git isaac; финальные артефакты копируются на ноутбук отдельно.

Приняты новая карта и один baseline pilot с 3 goals в собственной статичной сцене.
Не приняты active/control, runner с формальными seeds/reset/metadata, многократные
парные прогоны и APE/RPE, длительная надёжность, новые динамические препятствия,
универсальная точность боковых STag и колесная физика. Следующий этап — 008.
