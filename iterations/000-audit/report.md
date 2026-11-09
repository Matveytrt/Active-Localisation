> Исторический отчёт. Текущее состояние и следующий шаг — в [реестре](../../TASK_REGISTRY.md).

# Active-Localisation → Isaac Sim: обследование и план

Дата исходного обследования: 9 ноября 2026.

Обновление: шаги 1–2 выполнены, рабочие копии созданы на сервере и ноутбуке.
Доступ Docker получен; официальный Isaac Sim 5.1.0 запущен отдельно от команды,
проверены физика и передача /clock во внешний Humble-контейнер.
Подробности: [результат шага 1](../../isaac/STEP_1_RESULT.md).
Собрана своя навигация, прошли 35 C++-тестов и 169 Python-тестов;
импортирован собственный Astribot и получен реальный RTX-кадр.
См. [сборку](../../isaac/STEP_2_RESULT.md) и [GPU/модель](../../isaac/STEP_3_RESULT.md).
Шаг 3 частично выполнен: импорт и рендер проверены, длительная стабильность ещё нет.
Ниже сохранён исходный аудит; отмеченные в нём ограничения доступа уже сняты.

## Цель и границы

Воспроизвести решение Matveytrt/Active-Localisation на Isaac Sim для Astribot S1:
STag → RTAB-Map → Nav2 с собственными плагинами активной локализации,
сравнение baseline / active / control и оценка APE/RPE.

Из <isaac-runtime-root> используем только доступный runtime Isaac Sim / Isaac Lab.
Навигацию, мир, карты, ROS-мост и стартовые скрипты команды не принимаем за основу.
Файлы команды были только прочитаны для выяснения способа установки.
Gazebo-вариант своего проекта сохраняем как контрольный вариант.

## Что проверено

### GitHub

- Репозиторий: https://github.com/Matveytrt/Active-Localisation
- Ветка main, дерево коммита `6a21508cf86cc4b21d12d3e320f0f2f6a5357407` на момент проверки.
- Два отдельных контейнера ROS 2 Humble: gazebo и navigation.
- Связь через DDS/ROS 2 позволяет заменить симулятор без переноса алгоритма внутрь Isaac.
- `al_nav2_plugins`: LocalizabilityLayer и LocalizationAwarePlanner поверх Theta*.
- MPPI и omnidirectional base: требуется боковая скорость vy, а не только vx и yaw.
- Эталон алгоритма на Python: `active_localization`.
- `al_odom_noise`: /odom_gt → /odom и TF odom → base_link.
- `al_eval`: запись истинной траектории /odom_gt и оценки TF map → base_link.
- RTAB-Map использует лидар и landmarks. RGB/depth непосредственно в RTAB-Map отключены.
- Камера с названием rgbd фактически задана в Gazebo как RGB `type=camera`.
- `get_map.sh` содержит TODO-TAG; SHA-256 карты состоит из нулей. Автоматическая загрузка исходной карты сейчас не готова.
- В navigation/Dockerfile явно копируются не все новые пакеты: al_nav2_plugins,
  active_localization, al_eval, al_odom_noise требуют отдельной проверки сборки overlay.
- Экспериментальный runner привязан к Gazebo, startup sleeps и процессам Gazebo/RViz;
  простая замена команды запуска не завершает перенос runner.

### Удалённый ПК

| Параметр | Результат |
|---|---|
| SSH | Алиас `astribot`, вход по ключу проверен |
| Узел | Удалённый ПК |
| ОС | Ubuntu 24.04.4 LTS, x86_64 |
| CPU | Intel Core i5-14400, 16 логических CPU |
| GPU | RTX 5060 Ti, 16311 MiB VRAM |
| Драйвер | 580.178.04 |
| RAM | 15 GiB доступного системе объёма, около 10 GiB available при осмотре |
| Swap | 4 GiB |
| Диск | около 186 GiB свободно на общем разделе /, /srv, /home |
| ROS на хосте | /opt/ros отсутствует |
| Docker | установлен, демон работает, доступ к сокету запрещён текущему пользователю |
| Группы пользователя | sudo и external-runtime есть; docker отсутствует |
| sudo без пароля | недоступен |
| GPU compute-процессы | nvidia-smi не показал активных compute-процессов при осмотре |

Установка Isaac преимущественно контейнерная. Каталоги `isaac-sim` в /srv содержат
кэши, данные и логи, а не подтверждённую standalone-установку с исполняемым файлом.

Найдены:

- `<isaac-runtime-root>/isaaclab_docker/Dockerfile`: FROM nvcr.io/nvidia/isaac-lab:2.3.2,
  ROS Jazzy, собственная сборка ROS под внутренний Python 3.11.
- `<isaac-runtime-root>/astribot_ws/Astribot-Playground/docker/docker-compose.yml`:
  image `isaac_lab_astribot_ready`, container `isaaclab_astribot_container`.
- Другой compose ссылается на `isaaclab_astribot_image`. Какой образ реально установлен,
  пока не проверено из-за прав Docker.
- Исторические логи: App Version 5.1.0, команда запуска через
  `/workspace/isaaclab/_isaac_sim/kit/kit`.
- `run_isaac.sh` в корне <isaac-runtime-root> содержит заметки и фрагменты команд;
  его нельзя считать воспроизводимым стартовым скриптом.
- Некоторые старые compose используют ~/external-runtime, хотя данные находятся в <isaac-runtime-root>.

Версия Isaac Sim 5.1.0 подтверждена историческими логами, а не проверкой текущего образа.
Запуск на текущем GPU, состояние ROS Bridge и рабочая GUI/streaming-сессия не проверены.
Наличие tar-архива не доказывает наличие загруженного Docker-образа.

## Предлагаемая архитектура

```text
Отдельный Isaac-контейнер: свой мир + Astribot + сенсоры + ROS Bridge
    ↑ /cmd_vel (vx, vy, wz)
    ↓ /clock, /odom_gt, camera image/info, lidar clouds, robot TF/joint states
Свой ROS 2 Humble navigation-контейнер:
    lidar_stitching
    al_odom_noise → /odom + odom→base_link
    STag → tf_to_fiducial → RTAB-Map → map→odom
    Nav2 + al_nav2_plugins → /cmd_vel
    al_eval → траектории/детекции → существующий eval/
```

Рекомендуемый первый вариант: сохранить Humble в navigation; проверить встроенный
Humble backend Isaac ROS Bridge в отдельном процессе с чистым окружением.
NVIDIA документирует Humble и Jazzy для 5.1 и встроенные библиотеки Python 3.11.
Внешние ROS-ноды могут работать на штатном Python своего контейнера через DDS.
Это проектное решение, его работоспособность в найденном образе требует smoke test.
Не считать обмен Jazzy↔Humble гарантированным решением и не переносить Nav2 на Jazzy
одновременно с заменой симулятора. Если образ жёстко привязан к Jazzy, сначала оценить
отдельный чистый runtime из доступного базового образа.

Isaac Lab нужен как доступный способ получить runtime; RL и vectorized environments
для этой задачи не требуются. Достаточен standalone-сценарий Isaac Sim.

## Контракт симулятора

| Интерфейс | Требование |
|---|---|
| /cmd_vel | geometry_msgs/Twist; скорости в системе базы, поддержка vx/vy/wz; watchdog остановки |
| /clock | rosgraph_msgs/Clock, единственный источник sim time |
| /odom_gt | nav_msgs/Odometry, истинная поза и скорость, согласованные frames; ориентир 50 Hz |
| /odom | публикует только al_odom_noise, не Isaac |
| odom → base_link | публикует только al_odom_noise |
| map → odom | публикует только RTAB-Map |
| robot TF | геометрия собственного URDF; один владелец каждого ребра TF |
| /torso_rgbd/image_raw | sensor_msgs/Image, RGB 640×480, ориентир 30 Hz |
| /torso_rgbd/camera_info | согласованные K/P и frame_id; HFOV исходника 1.089 rad |
| lidar front/back points | sensor_msgs/PointCloud2, ориентир 10 Hz; сохранить собственные extrinsics |
| /lidar/points_joined | результат своего lidar_stitching, target_frame base_link |

Исходный передний лидар: 120 горизонтальных отсчётов, 16 вертикальных,
углы [0, 6.2832] и [-0.3, 1.0] rad, дальность 0.4–20 m, gaussian stddev 0.01 m.
При реализации отдельно сверить задний лидар и реализацию шума в Isaac.
Датчики ориентируем по URDF, не по внешнему виду модели.

Для первой версии воспроизвести кинематическую omnidirectional-модель Gazebo:
интегрирование команд в sim time и корректные препятствия/контакты.
Не подменять её differential drive; не включать физику колёс и манипуляторов до
получения сопоставимого результата. Проверить, что робот не проходит через стеллажи.

## Этапы и критерии готовности

1. **Доступ и инвентаризация runtime.** Получить разрешённый доступ к Docker;
   только чтение `images`, `ps`, `inspect`, версии Isaac/Python/ROS Bridge.
   Зафиксировать image ID/digest, доступность assets и способ запуска без автозапуска
   командного demo. Не останавливать контейнеры команды. Готово: точная команда запуска
   отдельного контейнера и сохранённый manifest версий.

2. **Изолированное рабочее место.** Предлагаемый путь на сервере:
   `/home/<remote-user>/projects/Active-Localisation`, ветка `isaac-migration`.
   Своё имя Compose project, контейнеров, логи, кэши, карты и проверенный свободный
   ROS_DOMAIN_ID. Взять точный commit собственного проекта. Новые файлы в `isaac/`,
   без монтирования всего workspace команды и без редактирования <isaac-runtime-root>.
   Готово: свой navigation собирается со всеми четырьмя новыми пакетами.

3. **Минимальный запуск Isaac.** Пустая сцена, один робот/примитив, ROS Bridge,
   /clock; запуск headless и сохранение диагностического кадра камеры.
   Проверить VRAM/RAM, RTF и ошибки рендера. После прогрева выдержать 5 минут без crash/OOM.
   16 GB RAM ниже официального минимума 32 GB: успех запуска заранее не обещать.
   При проблемах сначала упрощать сцену/рендеринг, а не менять алгоритм.

4. **Собственный Astribot и движение.** Импорт геометрии из своего URDF/meshes,
   исключение Gazebo plugins, сохранение frames и позы корпуса/головы.
   Тесты отдельно: vx, vy, wz, нулевая команда, watchdog, препятствие и reset.
   Готово: заданные перемещения и yaw совпадают с /odom_gt в пределах заранее
   зафиксированного допуска, оси и направления проверены численно.

5. **Сенсоры и ROS-контракт.** Добавить RGB + camera_info, затем два лидара и
   собственную сшивку; проверить QoS, timestamp, TF lookup и frames.
   Штатные backend/OmniGraph ROS-публикации предпочтительнее сборки всего ROS внутри Isaac.
   Готово: геометрический тест стены/маркера, корректные единицы и частоты, без TF-дублей;
   фактические частоты и RTF записаны, отклонения от исходника явно зафиксированы.

6. **Мир и маркеры своего эксперимента.** Источник истины —
   `astribot_hd15_hd17_walls.world`, модели стеллажей и текстуры HD15/HD17 из своего repo.
   Перенести позиции, масштабы, коллизии и marker IDs в USD/генератор сцены.
   Проверить метры, quaternions, ориентацию лица маркера и camera optical frame.
   Готово: STag из своего контейнера распознаёт реальные изображения, ID и физические
   размеры согласованы. Не заменять детекции идеальными позами маркеров из симулятора.

7. **Картирование и baseline.** Построить новую Isaac-карту своим RTAB-Map,
   экспортировать landmarks.yaml из той же базы, сохранить pristine DB и SHA-256.
   Не брать карты команды и не предполагать совместимость старой Gazebo DB.
   Запустить al_odom_noise сначала без шума, затем с фиксированным seed.
   Готово: baseline проходит короткий маршрут; GT и оценка разведены; проверено,
   что GT в world или spawn-relative frame, а map/world offset явно сохранён.

8. **Active/control и автоматизация.** Подключить неизменённые плагины, выполнить
   check_live.sh. Добавить backend Isaac в runner: запуск, ожидание /clock/сенсоров/TF,
   остановка только собственных процессов, reset всей сцены/шумов/SLAM и сохранение логов.
   Начать с полного перезапуска собственного процесса между эпизодами; оптимизировать
   reset позднее. Готово: baseline/active/control используют одну карту/сцену/start/seeds,
   а scorer и loc_layer различаются ровно по правилам собственного проекта.

9. **Воспроизведение эксперимента.** Сначала 3 парных seed на режим для проверки
   пайплайна; объём основного исследования определить после пилота.
   Сохранить APE/RPE translation/rotation, успех/время маршрута, recoveries, видимость
   маркеров, частоты сенсоров, RTF, ресурсы и версии. Использовать существующий eval/.
   Сравнивать режимы внутри Isaac; сравнение с Gazebo отдельно, без предположения
   одинакового шума, света, изображения или статистического результата.

## Что сейчас мешает следующему этапу

Для Docker inspect и запуска отдельного контейнера требуется действие с sudo:
например, администратор добавляет пользователя <remote-user> в группу docker,
после чего создаётся новая SSH-сессия. Это даёт широкие права на хосте; альтернативой
может быть запуск/inspect через sudo в интерактивной сессии пользователя.
Пароль не записываем в файлы и не передаём в командной строке.
До решения доступа можно готовить исходники/конфигурацию, но нельзя честно подтвердить
текущий образ, ROS Bridge и успешный запуск.

## Документация и основания

- Собственная архитектура: https://github.com/Matveytrt/Active-Localisation/blob/main/README.md
- Источник GT: https://github.com/Matveytrt/Active-Localisation/blob/main/gazebo/src/world_and_robot/urdf_astribot/gazebo_cmd_vel.xacro
- Шум: https://github.com/Matveytrt/Active-Localisation/blob/main/navigation/src/al_odom_noise/config/odom_noise.yaml
- Запись оценки: https://github.com/Matveytrt/Active-Localisation/blob/main/navigation/src/al_eval/config/traj_recorder.yaml
- Карта: https://github.com/Matveytrt/Active-Localisation/blob/main/scripts/get_map.sh
- Isaac 5.1 ROS/Python: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html
- Isaac 5.1 требования: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html

Документация NVIDIA отмечает 5.1 как неподдерживаемый релиз на дату просмотра.
Воспроизведение начинаем с обследования доступного runtime; обновление до другой
версии — отдельное решение после минимального теста, без обновления установки команды.
