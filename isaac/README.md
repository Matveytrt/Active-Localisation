# Isaac Sim: отдельный runtime Active-Localisation

Актуальный [реестр задачи](../TASK_REGISTRY.md); подробные [итерации](../iterations/README.md).

Рабочая ветка: `isaac`. Gazebo и navigation пока не изменены.

Своя навигация уже собрана и проверена: [результат этапа 2](STEP_2_RESULT.md).
GPU-рендер и собственная модель Astribot: [результат импорта](STEP_3_RESULT.md).
Для повторения сборки: `bash isaac/build_navigation.sh`,
для модульных проверок: `bash isaac/test_navigation.sh`.

## Движение базы и штатные датчики

Отдельная проверочная комната с собственным Astribot:

```bash
cd ~/projects/Active-Localisation
bash isaac/start_bridge.sh
docker logs -f al-isaac-astribot
```

Готовность — `AL_ASTRIBOT_BRIDGE_READY` и новый `isaac/runtime/bridge/ready.json`,
после появления настоящих RGB/depth и обоих native PhysX scans. Старт может
занять около двух минут; `app ready` раньше этого недостаточно.
По умолчанию runtime работает до 900 секунд wall time после готовности;
для другой длительности передать `--max-wall-seconds 1800` (`0` без ограничения).
Скрипт безопасно отказывается запускать второй собственный симулятор.

Публикуются `/clock`, `/odom_gt`, `/torso_rgbd/image_raw`, `/torso_rgbd/camera_info`,
`/torso_rgbd/depth/image_raw`, `/lidar_front/points`, `/lidar_back/points`,
`/isaac/usd_pose`, `/isaac/status`. Исходные noise/stitching запущены отдельными
собственными контейнерами и добавляют `/odom`, `/lidar/points_joined`, TF odom→base_link.
GT имеет frame world и не публикует идеальный TF. CameraInfo/изображения используют
torso_rgbd_optical; штатный body-frame torso_rgbd сохраняется для исходного STag.

Скорость базы: x вперёд, y влево (m/s), z вращения против часовой стрелки (rad/s).
Например, в отдельном SSH-терминале:

```bash
docker exec -it al-isaac-navigation bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"'
```

Ctrl+C прекращает команды; watchdog останавливает движение через 0.5 s sim time
или 2 s wall time. Скорости ограничены ±0.6 m/s по x/y и ±0.8 rad/s по yaw.
Текущая кинематическая модель использует footprint .65×.75 m и guard четырёх стен
диагностической комнаты. Это не физика колёс и не защита от произвольных meshes.

```bash
# Проверка внешними ROS-сообщениями; она самостоятельно двигает базу и делает reset.
bash isaac/verify_bridge.sh
# Согласованный сброс базы, затем модели шума; время не идёт назад.
bash isaac/control_bridge.sh reset
# Штатное завершение, затем дождаться остановки контейнера.
bash isaac/control_bridge.sh shutdown
docker wait al-isaac-astribot
```

Перед новым запуском сохранять нужные доказательства: файлы текущего прогона
перезаписываются. Приёмка — `isaac/runtime/navigation/bridge-verification/result.json`
со `status=passed`, кадр рядом; завершение — `isaac/runtime/bridge/result.json`
со `status=finished`, затем фактические exit/OOM контейнера. Логи — `docker logs`.
Единственный код выхода Kit недостаточен. Отчёт и ограничения —
[итерация 005](../iterations/005-base-sensors/report.md).

Заявленные частоты 50 Hz GT, 30 Hz RGB/depth, 10 Hz каждый lidar относятся к sim time.
Фактический RTF и время участков цикла записываются в `/isaac/status` и итоговый JSON;
эти частоты не означают работу в реальном времени. Реальная камера использует RTX,
лидары — native PhysX Generic с исходным паттерном 120×16 и шумом .01 m.
Знак native zenith адаптирован к исходному elevation Gazebo.

Для крупных сырых изображений все собственные участники используют `fastdds.xml`
с UDPv4 socket buffers 4 MiB. На проверенном сервере rmem_max/wmem_max уже 4 MiB;
системные настройки и чужие контейнеры не менялись. CameraInfo reliable совместим
с исходным STag; RGB/depth и облака сохраняют SensorData QoS.

## Собственный мир и STag

```bash
bash isaac/start_bridge.sh --scene own-world --max-wall-seconds 1800
bash isaac/verify_world.sh
```

Сценарий создаёт `isaac/runtime/world/own_world.usdc` из выбранного исходным launch
мира `astribot_hd15_hd17_walls.world`. Полные DAE стеллажей, source collision-boxes,
стены, пол и все 24 исходные PNG сохраняют собственные размеры и позы.
Штатный spawn — [-2,-.5,.16], yaw=0. Для отдельной диагностики его можно
переопределить `--spawn X Y YAW` (yaw в радианах). Reset возвращает к выбранному spawn.

Вместе с own-world запускаются `al-isaac-stag-hd15` и `al-isaac-stag-hd17`;
переключение обратно на room останавливает их, чтобы не дублировать TF.
Это исходные C++-детекторы с исходными параметрами и измеренными размерами
чёрных квадратов .064/.08 m. Код алгоритма и error correction не меняются.
Вход — реальные RGB/CameraInfo, выход — `torso_rgbd→stag_hd15_* / stag_hd17_*`.
Overlays: `/isaac/stag/stag_hd15/detection_image_hd15` и аналогично hd17.

Исходный noise config `init_from_gt=true` сохраняется: GT и noisy odom стартуют
с исходных x/y. Начальное численное совмещение world/odom не является идеальным
TF world→odom; такого TF симулятор не публикует. Будущий map→odom — только RTAB-Map.
SAT-guard учитывает проекции всех 34 ориентированных source boxes; плоский пол
остаётся отдельным PhysX collider. Физика колёс по-прежнему не моделируется.

Результат внешней проверки — `isaac/runtime/navigation/world-verification/result.json`;
рядом реальные кадры, overlays и контрольный кадр пустой стены. Source poses
используются только проверкой для оценки, не входят во вход детектора.
Принимается representative frontal PnP обеих семей, облака, source transforms,
reset и столкновения. Боковые ориентации IPPE и ложные сырые ID остаются ограничениями;
полный SLAM ещё не принят. Перед RTAB-Map сохранять исходный allowlist HD17 1–5
в tf_to_fiducial: ID 0 не входит в граф, внесценовые 116/202 не должны попадать в landmarks.
Не запускать старый complete_system.launch.py поверх этих детекторов: он создаст ещё два.

`world_source.py` проверяет исходный `astribot_hd15_hd17_walls.world`, композицию поз,
assets/хеши и фактические размеры рисунка STag. Запуск в собственном ROS-контейнере:

```bash
docker exec al-isaac-navigation python3 /repo/isaac/world_source.py --output /workspace/world-source-manifest.json
```

Подробные границы приёмки, включая расхождение HD15 .06/.064 m, —
[итерация 006](../iterations/006-world-stag/report.md).

## Своя карта и baseline

[Итерация 007](../iterations/007-map-baseline/report.md): новая карта из настоящих
сенсоров/noisy odom и 3 успешные NavigateToPose цели в свежей локализации.
Это один pilot; active/control и статистические эксперименты ещё впереди.

На сервере, с новым именем DB для каждого mapping:

```bash
bash isaac/start_bridge.sh --scene own-world --max-wall-seconds 7200
bash isaac/start_slam.sh mapping /workspace/maps/OWN.db
bash isaac/run_mapping_route.sh
bash isaac/save_mapping.sh /workspace/maps/OWN.db
bash isaac/control_bridge.sh shutdown
docker wait al-isaac-astribot
bash isaac/finalize_mapping.sh /workspace/maps/OWN.db /workspace/maps/OWN.global.db
```

Online SLAM ограничивает WM до 200 nodes, фильтрует occupancy cache по .15 m/10°,
не загружает весь local-grid cache заранее. Это общий ресурсный профиль Isaac
для 16 GiB RAM, явно отличающийся от unlimited defaults исходника. Он не удаляет
исторические наблюдения: они остаются в DB/LTM. Без него первый долгий mapping
получил OOM. В конце native RTAB-Map global optimizer включает LTM на отдельной
копии DB при выключенном симуляторе; штатный exporter читает полный optimized graph.
Побайтовая проверка Data/Link/raw poses подтвердила сохранение наблюдений.

Принятая карта: `isaac/runtime/navigation/maps/isaac-20261109-global.db`,
landmarks рядом в `.landmarks.yaml`. Occupancy `.pgm/.yaml` сохранены с префиксом
`isaac-20261109-bounded`. SHA pristine — в `.db.sha256`; карты и логи вне Git.
Детальная оценка — `.evaluation.json`, рисунок —
`isaac/runtime/navigation/mapping-007/map-and-trajectory.png`.

Для baseline сначала сделать отдельную writable DB-копию в `/workspace/maps`
через собственный navigation контейнер, затем полностью перезапустить simulator
и noise/STag, запустить `start_slam.sh localization /workspace/maps/EPISODE.db`.
После `AL_ASTRIBOT_BRIDGE_READY`:

```bash
bash isaac/start_baseline.sh
bash isaac/verify_baseline.sh
```

Probe ждёт active lifecycle nodes, использует известный start seed /initialpose
один раз и посылает только NavigateToPose goals. Ручных Twist во время этих целей
нет. Baseline — исходные Theta*/MPPI/BT, общий override скорости .6/.6/.8 согласует
контроллер с физическим clamp адаптера. Будущие active/control должны делить эти
ограничения, ресурсный профиль, pristine карту и новый landmarks YAML.

Остановка: сначала `docker compose -f isaac/compose.yaml --profile nav --profile slam stop baseline slam fiducial`,
затем `control_bridge.sh shutdown`, дождаться Docker, остановить свои noise/stitching/STag.
Nav2 launch получает SIGINT для clean shutdown; core RTAB-Map запускается напрямую
и сохраняет DB при завершении.

## Импорт собственной модели

`bash isaac/run_import.sh` запускает отдельный контейнер `al-isaac-robot-import`.
`prepare_robot.py` берёт URDF, meshes и позу только из собственного репозитория;
исходные файлы не меняются. В `isaac/runtime/robot` сохраняются фиксированный URDF,
полное дерево transforms, USD, изображение и `import_result.json`.

RTX инициализируется асинхронно: сообщение Kit «app ready» ещё не означает готовность
рендера. Импорт ждёт подключения RTX, а затем валидного кадра камеры, с отдельными
таймаутами 180 секунд. Наличие GPU проверено CUDA-вычислением в контейнере;
Vulkan хоста также видит RTX 5060 Ti и поддержку ray tracing.
Включение физического монитора для headless-запуска не требуется.

Перед повторным запуском сохранить нужные логи и удалить только завершённый
контейнер `docker rm al-isaac-robot-import`. Кэши и USD не являются файлами Git.
Этот импорт проверяет геометрию и внешний вид; управление базой и штатные сенсоры
робота добавляются отдельно.

## Копии проекта

- Сервер: `/home/<remote-user>/projects/Active-Localisation`.
- Ноутбук: `<local-home>\Documents\AstribotIsaac\Active-Localisation`.
- GitHub остаётся `origin`. Локальный remote `workstation` указывает на сервер.
- Основная среда исполнения — сервер. После завершённого изменения сохраняем commit
  на сервере и подтягиваем его на ноутбук: `git fetch workstation`,
  `git merge --ff-only workstation/isaac`.
- Перед началом работы проверяем `git status` в обеих копиях. Изменения одновременно
  в двух местах автоматически не перезаписываем; ветки не обновляем с force.
- Кэши, runtime-логи и большие карты не коммитим. Нужные результаты отдельно копируем
  на ноутбук. Автоматическая фоновая синхронизация не настроена.
- На ноутбуке можно выполнить `powershell -File isaac/sync_from_remote.ps1`:
  скрипт проверит отсутствие локальных правок и подтянет commit сервера без force.

## Первый запуск

Используется уже имеющийся официальный образ NVIDIA Isaac Sim 5.1.0 по digest,
а не образ и код Astribot команды. Собственные контейнеры имеют префикс `al-isaac-`.
ROS_DOMAIN_ID=71 отличается от найденного у команды ID=20. ROS среды вне нашего
контроля могут использовать другие ID, поэтому это не гарантия глобальной уникальности.
Перед полноценными экспериментами проверить отсутствие чужих publishers в домене.

На сервере:

```bash
cd ~/projects/Active-Localisation
bash isaac/run_smoke.sh
docker logs -f al-isaac-runtime-smoke
```

Тест создаёт пол и падающий куб, включает Humble ROS Bridge и публикует `/clock`.
После старта выполняет около 30 секунд симуляции. Это минимальная проверка runtime,
не проверка Astribot, камеры, лидаров, Nav2 или длительной стабильности.

Успех определяется по `isaac/runtime/smoke_result.json` со `status: passed`,
а не только по коду выхода Docker: завершение Kit может скрывать ошибки Python.
Частота шага/RTF здесь не являются benchmark производительности будущей сцены.

Параллельно, пока симулятор работает, проверить получение времени извне:

```bash
docker run --rm --name al-isaac-clock-probe --network host \
  -e ROS_DOMAIN_ID=71 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$PWD:/work:ro" osrf/ros:humble-desktop-full \
  bash -lc 'source /opt/ros/humble/setup.bash; python3 /work/isaac/verify_clock.py'
```

Контейнер probe надо запускать до завершения smoke. Таймаут ожидания — 180 секунд.
Скрипт требует три строго возрастающих значения `/clock` из отдельного контейнера.

Для повторного запуска сначала сохранить нужные логи и удалить только завершённый
тестовый контейнер `docker rm al-isaac-runtime-smoke`. Скрипт не удаляет контейнер
автоматически и не трогает контейнеры команды.

Файлы runtime создаются контейнером от root. Код проекта создаётся SSH-пользователем;
не делать рекурсивный chown каталога команды ради кэшей.

## Единый эксперимент Isaac

Запускать на удалённом ПК из корня репозитория, когда собственные симулятор,
SLAM и Nav2 остановлены:

```bash
python3 -u isaac/run_experiment.py --batch UNIQUE_NAME --seed 42
```

По умолчанию последовательно baseline, active, control. Можно выбрать
`--modes baseline active control`; порядок не меняет входную карту/start/seeds.
Runner использует принятую pristine карту 007, исходный `shelves_south_wall`,
отдельную writable DB на эпизод, полный restart Isaac/ROS и один фиксированный
initial pose. Общие ограничения скорости и bounded SLAM одинаковы. MPPI RNG
не управляется, поэтому одинаковые seeds не обещают побитового повтора движения.

Файлы результата: `isaac/runtime/navigation/experiments/UNIQUE_NAME/`;
DB-копии: `isaac/runtime/navigation/maps/episodes/UNIQUE_NAME/`.
Lock, checksum, проверка свободного места, занятых собственных контейнеров и отказ
от перезаписи предшествуют запуску. Чужие контейнеры не используются и не останавливаются.
После трёх успешных режимов:

```bash
docker exec al-isaac-navigation bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /repo/isaac/analyze_experiment.py /workspace/experiments/UNIQUE_NAME'
```

Проверка TUM/реальных маршрутов пишет `analysis.json` и `comparison.png`.
Успешный pilot доказывает работоспособность трёх режимов; парная статистика и
формальная APE/RPE относятся к следующему этапу.

## Плановая оценка времени

Предварительная оценка активной работы с отладкой; время ожидания пользователя,
скачиваний и недоступности ПК может увеличить календарный срок.

| Этап | Время |
|---|---|
| 1. Доступ, точные версии, запуск и проверка ROS-моста | 0.5–1.5 ч |
| 2. Изоляция репозитория и сборка своей navigation | 1–3 ч |
| 3. Минимальная сцена, рендеринг и проверка ресурсов | 1–3 ч |
| 4. Собственный Astribot и omnidirectional-движение | 4–8 ч |
| 5. Камера, два лидара, TF и полный ROS-контракт | 8–16 ч |
| 6. Собственный мир и STag-маркеры | 4–8 ч |
| 7. Новая карта RTAB-Map и baseline | 4–8 ч |
| 8. Active/control и адаптация runner | 4–8 ч |
| 9. Первые парные прогоны и оценка | 3–8 ч |

Итого ориентировочно 30–64 часа активной работы. С резервом на интеграцию —
примерно 5–10 рабочих дней. Первое движение с сенсорами — ориентир 1–3 рабочих дня.
Расширенная статистика для отчёта и серьёзная переработка физики колёс сюда не входят.
Главные неопределённости: RAM 16 GB, геометрия/frames модели, изображения STag,
соответствие лидара и состояние исходной сборки navigation.

Методика ROS clock:
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html
