# Реестр задачи: Active-Localisation → Isaac Sim

Обновлён: 9 ноября 2026, Europe/Moscow. Это единственный актуальный документ прогресса и планов.
Подробности и исторические доказательства — в [iterations](iterations/README.md).
Правила сопровождения — в [AGENTS.md](AGENTS.md).

## Цель

Воспроизвести собственное решение Matveytrt/Active-Localisation для Astribot S1 в Isaac Sim:
свой мир со стеллажами и STag, RTAB-Map, Nav2 с активной локализацией,
сравнение baseline / active / control и APE/RPE. Gazebo-вариант сохраняем для сравнения.

## Инструкции пользователя и принятые решения

- Используем только установленный Isaac runtime. Не переносим стороннее решение, его карты или навигационный стек.
- Все тяжёлые операции выполняем на удалённом ПК; ноутбук используется для копии исходников и артефактов.
- Ветка задачи — **isaac** в [репозитории](https://github.com/Matveytrt/Active-Localisation).
- Реестр читается перед работой и обновляется после шага; подробный отчёт и ревью каждого шага хранятся в iterations/.
- Сохраняем ROS 2 Humble и собственные алгоритмы; не совмещаем замену симулятора с миграцией Nav2 на Jazzy.
- База omnidirectional: поддержка vx, vy, wz обязательна. Для первого воспроизведения сохраняем фиксированную позу и кинематическую модель Gazebo.
- Истинная /odom_gt отделена от шумной /odom. TF odom→base_link публикует только al_odom_noise, map→odom — только RTAB-Map.
- STag должен работать по реальному изображению. Идеальные позы маркеров из симулятора не подменяют детекции.
- Карта и landmarks должны соответствовать собственному миру Isaac; командные карты не используем.
- Минимальные smoke tests не считаем полноценным воспроизведением эксперимента.

## Где работаем

| Объект | Значение |
|---|---|
| SSH | Алиас `astribot`; вход по ключу |
| Сервер | `~/projects/Active-Localisation` |
| Ноутбук | `Documents/AstribotIsaac/Active-Localisation` внутри домашней папки |
| Ветка | isaac |
| Старая ветка | isaac-migration сохранена, больше не используется для новых шагов |
| База исходного main | 6a21508cf86cc4b21d12d3e320f0f2f6a5357407 |
| Код перед введением реестра | f52246c9780a133153576f896400ba104824a3c2 |
| Правила синхронизации | commit на сервере → GitHub isaac → ноутбук; только обычные обновления |
| Локальный helper | isaac/sync_from_remote.ps1; подтягивает сохранённую ветку сервера |

## Окружение — проверенные факты

- Ubuntu 24.04.4, i5-14400, RTX 5060 Ti 16 GB, драйвер 580.178.04, RAM 16 GB.
- Docker доступен рабочему пользователю через группу docker; sudo остаётся парольным.
- Официальный Isaac Sim 5.1.0, VERSION 5.1.0-rc.19+release.26219.9c81211b.gl, Python 3.11.13.
- Образ Isaac закреплён digest sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54.
- Свой образ navigation: al-isaac-navigation:humble; Compose project al-isaac, контейнер al-isaac-navigation.
- ROS_DOMAIN_ID=71, rmw_fastrtps_cpp, UDPv4. У найденного контейнера команды domain 20; перед экспериментами проверить отсутствие посторонних publishers.
- Исходники navigation в контейнере read-only /repo; build/install в isaac/runtime/navigation.
- GPU проверен CUDA-вычислением и реальным RTX-кадром; монитор для успешного headless-запуска не понадобился.
- Kit «app ready» наступает раньше готовности RTX. Ждём подключённый engine и валидный кадр; в import_robot.py два таймаута по 180 секунд.

## Подтверждённый прогресс

### Текущее состояние после запуска 09.11.2026, 11:23 МСК

Разовое отложенное выполнение `astribot-isaac-sim` состоялось: 005 принята в
пределах диагностической комнаты. После прямой просьбы выполнить ещё один этап
сейчас (около 08:37 МСК) завершена 006: собственный мир, обе семьи реальных STag,
лидары, OBB-guard, исходный spawn и reset, с ограничениями качества PnP ниже.
Ранее отдельные движение и датчики объединены в 005. Пользовательский «3-й шаг»
сейчас означает следующий этап собственного мира и STag, а не повтор импорта 003.
Разовое выполнение `astribot-8` в 11:23:48 МСК состоялось: **007 принята** как
собственная новая карта и один настоящий baseline pilot с тремя NavigateToPose goals.
По последующей прямой просьбе выполнена **008**: общий runner и paired pilot
baseline/active/control, все 6 goals succeeded, реальный active effect проверен.
009 завершена:30 runs исходного протокола, Nav2 90/90 goals, APE/RPE и отдельный
итоговый отчёт о процессе переноса. Исключения: strict pose89/90, runner28/30;
подробности ниже. Новых автоматических запусков не назначали.
Подробная старая точка остановки сохранена как история в отчёте 005;
она не является текущим списком неисправностей.

| Итерация | Результат | Статус |
|---|---|---|
| [000](iterations/000-audit/report.md) | Исследован проект и сервер, выбран способ переноса | Завершена как аудит |
| [001](iterations/001-runtime/report.md) | SSH/Docker, отдельный runtime, физика куба и внешний /clock | Принято в пределах smoke test |
| [002](iterations/002-navigation/report.md) | Собраны 11 пакетов; 35 C++-тестов, 169 Python-тестов, 1 skip | Принято; ROS-интеграция впереди |
| [003](iterations/003-robot-render/report.md) | Свой Astribot: 38 links, 37 fixed joints, 18 mesh-файлов, RTX-кадр 640×480 | Импорт/рендер приняты |
| [004](iterations/004-task-registry/report.md) | Единый реестр, отчёты/ревью и ветка isaac | Документация и организация работы |
| [005](iterations/005-base-sensors/report.md) | vx/vy/wz, GT/noisy odom, RTX RGB/depth, два PhysX-лидара и сшивка, TF/watchdog/reset/стена | Внешняя ROS-проверка passed; диагностическая комната |
| [006](iterations/006-world-stag/report.md) | Полный собственный USD: 6 стеллажей, 24 STag, 4 стены; настоящие HD15/HD17, lidar, OBB/reset и отрицательный контроль | Внешний ROS-тест passed; PnP принят на фронтальных целях, ограничения ниже |
| [007](iterations/007-map-baseline/report.md) | Новая sensor DB, 947 узлов, 5 optimized HD17; свежая локализация и 3 Nav2 goals | Passed: собственная карта и один baseline pilot; полные эксперименты впереди |
| [008](iterations/008-runner-modes/report.md) | Общий runner, реальные baseline/active/control, seeds/reset/DB copies/артефакты | Passed: один paired pilot, 6/6 goals, active effect проверен; статистика впереди |
| [009](iterations/009-reproduction/report.md) | Основной shelves_loop_a, noise×4/×6, seeds42–46, 30 runs, APE/RPE/CI и отчёт о переносе | Завершено: actions90/90, strict pose89/90, runner28/30; исключения сохранены |

Код, USD-модель, кадр и логи продублированы на ноутбук; SHA-256 основного USD совпал.
Последний симулятор после теста завершился штатно (exit=0, OOM=false).
Собственные odom-noise/lidar-stitching, stag-hd15/hd17, fiducial/slam/baseline/recorder/episode остановлены;
al-isaac-navigation готов к работе. Командные контейнеры не меняли.
Это снимок состояния: перед следующим запуском проверить фактические контейнеры.

Итог 005: /odom_gt 50.03 Hz, RGB/depth 30.02 Hz, оба лидара и сшивка 10.01 Hz
по sim time. Внешний RTF=0.993; весь финальный runtime 42.77 s sim / 43.03 s wall.
Камера 640×480, fx=528.4337, optical-frame, depth до мишени 2.42 m.
Лидары 120×16, конечных 1347/1330, p99 surface error около 2 cm; сшивка 2232 точки.
Движение, остановка при x=3.67 перед стеной, reset и единственный TF-владелец проверены.
35 C++ tests; 175 Python passed, 1 skip; ещё 2 теста аудита 006 прошли отдельно.
Доказательства: isaac/runtime/navigation/bridge-verification/result.json и PNG,
isaac/runtime/evidence/bridge-accepted-runtime.json/.log, test-navigation-005.log.
Рендер проверен на GPU; исходная причина RTF≈0.069 была в Python-проверках bytes
ROS Image.data, исправлена штатным array.array('B') fast path. Допуски не снижались.

Итог 006: source USD содержит 35 точных collider transforms (ошибка 0),
24 texture faces, 12 mesh parts / 426288 shelf triangles. Собственный мир:
171.12 s sim / 172.41 s wall, внешний RTF .992. Реальный frontend RGB→STag→TF:
HD15 ID2 median translation 7.8 mm, normal 8.74°; HD17 ID0 3.2 mm и 2.12°.
Это фронтальные контрольные цели, не метрология всех 24 маркеров во всех ракурсах.
Лидары: 1520/1443 finite, p99 до source collisions 2.06/2.24 cm.
Стеллаж/стена остановили базу; пустая стена не дала свежих marker TF.
35 C++ +179 Python passed, 1 skip; после изменений прежний room ROS-test
также passed (GT/RGB/lidar 50/30/10 Hz, RTF .994). Оба runtime exit=0/OOM=false.
Доказательства 006: isaac/runtime/world/own_world.usdc и manifest.json;
isaac/runtime/navigation/world-verification/{result.json,PNG};
isaac/runtime/evidence/world-verification-final.*, world-runtime-final.*,
bridge-regression-006.*, test-navigation-006.log. Артефакты зеркалируются на ноутбук, вне Git.

Важные границы 006: исходный IPPE даёт скачки боковой normal около 42–44°;
сырые TF иногда содержат внесценовые HD17 116 / HD15 202. Алгоритм не меняли.
Это зафиксированные ограничения, не принятые корректные наблюдения. Существующий
tf_to_fiducial allowlist HD17 [1..5] нужно сохранить и проверить в 007;
HD17 ID0 не допускается RTAB-Map, контроль 007 обязательно с ненулевым ID.
Исходный noise init_from_gt=true сохранён: начальные GT/noisy x/y [-2,-.5],
world/odom численно совмещены в начале без публикации идеального world→odom TF.

Итог 007: реальный ненулевой HD17 ID1 проверен с ошибкой 9.27 mm;
/landmark_detections содержал только IDs 1–5, stale-negative control прошёл
после 1 s выдержки (exact .3 s threshold отдельно не измерялся).
Новая online DB 947 nodes/219 landmark links. Bounded WM/cache необходимы:
первый unlimited прогон вызвал OOM Isaac. Общий профиль: MemoryThr=200,
map_filter_radius=.15, map_filter_angle=10°, map_cache_loaded_on_init=false.
Пиковая cgroup memory bounded SLAM 1.81 GiB. Эти overrides явные, общие для режимов.
diagnostic_updater в своём ROS image обновлён 4.0.6→4.0.7 из-за ABI RTAB-Map 0.23.13.

Полный optimized граф с LTM консолидируется offline **в копию DB** native RTAB-Map
при остановленном симуляторе (isaac/finalize_mapping.sh); обычный Admin cache
bounded WM содержал только один landmark. Получены все 5 optimized landmarks,
median source-face XY error .0174 m. Hashes Data/Link/raw Node poses до/после равны;
ни GT, ни source poses не читаются optimizer, exporter проекта не изменён.

Принятая pristine карта: isaac/runtime/navigation/maps/isaac-20261109-global.db
(SHA-256 70a236237a60e4c4294d31ab308f449bdaa1e5be5dab5399742701a8ed6397fe).
Landmarks: тот же prefix .landmarks.yaml; occupancy .pgm/.yaml — prefix
isaac-20261109-bounded. В baseline — отдельная DB-копия, pristine hash не изменился.
Свежий полный Isaac/noise/STag reset, известный начальный /initialpose один раз;
дальше только Nav2 commands. Три goals SUCCEEDED: GT XY errors 6.24/1.04/8.93 cm,
estimated XY ≤8.97 cm, yaw ≤.0432 rad. TF/map/odom owners проверены.
35 C++ +179 Python passed, 1 skip; native finalizer replay/negative controls passed.
Mapping/localization simulator и RTAB-Map exit0/OOM=false. Nav2 SIGTERM cleanup
сначала потребовал kill; stop_signal заменён SIGINT, отдельный cleanup test exit0/OOM=false.

Артефакты 007: runtime/navigation/mapping-007 и baseline-007 (CSV/JSON/PNG),
runtime/navigation/maps (свои DB/YAML/PGM), runtime/evidence/*007*.
Один mapping pilot дал sampled XY RMSE map estimate .0592 m / noisy odom .3251 m;
это diagnostic latest-sample сравнение, не APE/RPE и не статистика преимуществ.

Итог 008: новый runner `python3 isaac/run_experiment.py --batch UNIQUE --seed 42`
последовательно запускает три режима. Один pristine DB SHA и новый exported YAML,
fixed spawn [-2,-.5,.16,yaw0], один initialpose, noise42/lidars42/43, полный restart,
отдельные writable DB. Исходные algorithms сохранены; control ровно 3 deltas,
общий .6/.6/.8 velocity/bounded SLAM. Runtime planner/layer загрузили5 markers;
control null/disabled проверен ROS parameters. MPPI RNG uncontrolled.

Финальный `runtime/navigation/experiments/pilot-008-b`: 6/6 goals succeeded,
recoveries0, GT XY max .08097m, estimated XY max .07388m, yaw max .04697rad.
Navigation RTF baseline/active/control .9641/.9692/.9674. Active/control на общих
10cm spatial bins x∈[-1,1.5): route y delta .443895m, heading delta .951849rad.
Active30 реальных accepted HD17 IDs1/2, остальные0. Это подтверждает работу
objective, но не статистический выигрыш; active wall time выше baseline.
Все 10 контейнеров каждого run exit0/OOM=false, pristine SHA неизменен.

Первый failed `pilot-008-a` сохранён: goals прошли, cleanup нет. Исправлены
ROS process wrappers, SIGINT/rclpy shutdown, sender изолирован отдельным контейнером.
35 C++ +190 Python passed/1skip; отдельно9 seed CLI passed, live lock guard passed.
TUM finite/unit quaternions/strict timestamps и clock audit passed. Reports,
configs/hash/version/results/logs/trajectory/PNG сохранены и зеркалируются на ноутбук;
writable episode DB остаются на удалённом ПК, canonical DB есть в обеих копиях.
Отдельным commit 3c37481 скрыты детали подключения в текущей документации;
исторические коммиты этим не очищены. SSH-настройки остаются вне репозитория.

Итог009: серия `reproduction-009`, runtime core commit44d02d0 зафиксирован до
начала и не менялся (только промежуточные updates реестра). Все30 planned jobs и90
actions учтены, без selective reruns/tuning. Map/landmark/scenario SHA, live noise
profiles/seeds/Nav2 configs/plugins, spawn/clock/TF и source provenance проверены.
GT идёт в исходную модель шума и evaluation, не напрямую в SLAM/Nav2.
30/30 data/input audits passed, TUM finite/unit/monotone, 100% association;
evo1.38.0, XY/no align/interpolation .02s/RPE1m, только goal window.

Навигация90/90 succeeded. Strict pose gate89/90: control×6 seed44, yaw snapshot
.1665058rad при допуске.10; причина отличия от внутреннего Nav2 check не установлена.
Runner28/30: второй failed — control×4 seed45, recorder RuntimeError при SIGINT
**после** конца окна. Файлы/counters/time coverage полны, данные включены в metrics;
teardown29/30, OOM/kill0. Код записи не меняет алгоритм TrajRecorder: после серии
добавлена управляемая остановка signal→spin_once→finalize→shutdown, fixture20/20passed.
Это не выдаётся за повтор30 runs после исправления или30 полностью зелёных gates.

APE RMSE mm mean±std baseline/active/control: ×4 37.96±1.71/34.42±.90/36.12±2.07;
×6 53.52±4.69/47.60±1.14/51.31±2.78. Active повысил path-with-detection на36.38/40.76
percentage points против baseline, добавил11.89/11.90s sim time. Active-minus-control
APE CI×4 пересекает0; RPE×4 active хуже control. Полные paired CI и все counterexamples:
[evaluation](report/isaac/evaluation.md). Не заявлять универсальное преимущество метода.
Raw pre-seed bad_db_init flags30/30 сохранены; post-initialpose max errors1.331mm/.000323rad.

Проверки009:35 C++ +208 Python/1skip; original eval31 +2real-evo passed;
extra3 path metric tests passed; native analytic controls и20 shutdown tests passed.
[Итоговый PDF](output/pdf/IsaacReproduction.pdf),6 страниц, и
[editable source](report/isaac/reproduction.md) описывают именно воспроизведение
робота/сцены/датчиков/карты/протокола, трудности и решения, без настройки личного ПК.
Исходный Gazebo отчёт не переписывался. Подробный009 report и явное саморевью сохранены.
Серия/evaluation/logs/controls зеркалируются на ноутбук; mutable DB30 runs остаются
на удалённом ПК, canonical DB есть в обеих копиях. Runtime вне Git.

## Что остаётся ограничением / расширением задачи

- Обобщение на новые сцены/seeds, абляции ICP и универсальная эффективность active localization.
- Причина расхождения yaw после Nav2 success в одном stress-run; данные и исходный gate сохранены.
- Точность боковых PnP и отсутствие raw false positives; allowlist живого SLAM уже проверен.
- Побитовая детерминированность (MPPI RNG не управляется), live timeout fault injection активной Nav2 цели.
- Длительная непрерывная стабильность:30 независимых restart-эпизодов не заменяют один многочасовой runtime.
- Физика колёс/контактная динамика: текущая база кинематическая, поза тела фиксирована как в Gazebo.
- Старый get_map.sh содержит заглушки URL/SHA; загрузка исходной базы не готова.

## Текущая точка завершения

По отдельной просьбе запущена010 — короткая видеозапись нового active demo
`demo-010-active` (shelves_south_wall, seed42, noise×4). Слева реальный torso RGB,
справа occupancy-map и GT overlay; это не часть серии009 и не её замена.
Ролик: `isaac/runtime/navigation/videos/experiment-active-010.mp4`;
готово:353frames/15fps/23.533s/H264/1280×720, обе Nav2 goals и pose checks passed.
Все кадры декодированы, средний просмотрен, файл зеркалируется на ноутбук;
video/recorder/экспериментальные контейнеры exit0/OOM=false. Отчёт/саморевью010 сохранены.

Этапы000–009 выполнены в описанных пределах. Задача воспроизведения исходного
навигационного протокола и отдельный отчёт завершены; исключения не скрыты.
Новые серии/физика/абляции — отдельные исследования по следующей просьбе пользователя.
Фактический dataset: `isaac/runtime/navigation/series/reproduction-009/manifest.json`
и `evaluation/summary.json`. Перед будущим запуском проверить актуальные Git/контейнеры;
не перезаписывать эту серию и не заменять её неудачные эпизоды новыми.

Ориентир первоначального планирования: 30–64 часа активной работы, с резервом 5–10 рабочих дней.
Это оценка, не обязательство и не основание останавливать работу. Пересматривать по фактическим результатам.

## Как продолжать после смены контекста

Прочитать этот реестр → report.md и review.md последней итерации → проверить Git/контейнеры →
выполнить следующую итерацию → сохранить доказательства → обновить реестр и коммит.
Исторический [план обследования](iterations/000-audit/report.md) не заменяет текущее состояние выше.
