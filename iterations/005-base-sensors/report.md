# Итерация 005 — движение базы, камера и два лидара

Дата: 9 ноября 2026. Статус: принята в пределах диагностической комнаты после внешней ROS-проверки.

## Реализация

Собственный isaac/astribot_bridge.py использует импортированный Astribot с фиксированной
позой, кинематический SE(2)-адаптер /cmd_vel, истинную /odom_gt, RTX RGB/depth,
и native PhysX Generic batch queries для двух 3D-лидаров. Исполнение только на удалённом ПК.

Сканирование и калибровка читаются из собственных исходных xacro, не из решения команды.
RGB 640×480, HFOV 1.089 rad, 30 Hz; лидары 120×16, 10 Hz, 0.4–20 m,
Gaussian range noise sigma=0.01 m. Все частоты относятся к sim time.
Добавлен стандартный torso_rgbd_optical; исходный STag остаётся в body-frame torso_rgbd
и продолжает делать своё существующее optical→body преобразование.

Проверочная комната состоит из четырёх физических стен и пола. Кинематическая база
использует тот же footprint 0.65×0.75 m, что Nav2, с SAT и подшагами против tunnelling.
Геометрия робота служит визуальной моделью; физика колёс не моделируется.
Коллайдеры собственного робота исключены из lidar raycasts. Цветные мишени —
только диагностическая визуализация, не физические препятствия и не STag.

Сохранён исходный al_odom_noise. Тонкая оболочка nav_support.py добавляет явный reset
его модели/RNG. /isaac/reset сбрасывает робота и команды; затем вызывается
/isaac/reset_noise. /clock остаётся монотонным, что сохраняет согласованность TF.
Для полностью нового эксперимента будущий runner будет перезапускать весь свой стек.

## Критерии, зафиксированные до живого теста

- Команды вперёд/вбок/поворот и ход при повёрнутой базе: ошибка ≤0.03 m/rad
  относительно интегрирования команды по фактической длительности sim time.
- Watchdog: после одного vx=0.2 m/s пройдено <0.13 m, затем неподвижность ≤0.001 m.
- Стена x=4 m: база останавливается при 3.64 < x ≤3.675 m, не проходит сквозь неё.
- Reset: GT и начальная шумная поза возвращаются к нулю с допуском 1e-7.
- CameraInfo: fx/fy отличаются от исходного HFOV менее чем на 0.01 px.
- Проекции трёх мишеней: ошибка центра <7 px; depth до мишени 2.42±0.04 m.
- Оба PointCloud2: 120×16, правильные frames, >800 конечных точек;
  99-й процентиль ошибки до поверхностей комнаты <0.04 m.
- Сшитое облако в base_link, >1500 точек. Динамический TF только odom→base_link
  от исходного noise node, 38 статических рёбер включая optical-frame.
- Частоты по stamps: GT 45–55 Hz, RGB 25–35 Hz, каждый лидар 8–12 Hz;
  stamps не идут назад. Wall-clock частоты и RTF рассматриваются отдельно.

## Проверки и результат

Пять модульных проверок bridge_core прошли: SE(2), оси, swept collision,
watchdogs/reset, quaternion-конверсия и параметры сканирования из исходников.
Живые проверки выполняются verify_bridge.py из отдельного ROS-контейнера.
Окончательный результат и пути доказательств будут добавлены после выполнения.

## Точка продолжения — остановка по просьбе пользователя

09.11.2026 около 03:18 МСК пользователь попросил продолжить ровно через 3 часа,
в том же запуске начать следующий этап собственного мира со STag. Автоматизация
astribot-isaac-sim создана на 06:17:48 МСК разово, текущий чат.
До её запуска дальнейшую реализацию не выполняем.

Код незакоммичен в isaac на обеих машинах: astribot_bridge.py, bridge_core.py,
physx_lidars.py, nav_support.py, start_bridge.sh, verify_bridge.py,
test_bridge_core.py и compose.yaml. HEAD перед работой d3ffe15e3f71eff13ad3175edcd4d0615016b41b.
Этот отчёт/ревью и реестр скопировать на сервер. Не терять незакоммиченную работу.

### Уже измерено — скалярный прототип

- 5 модульных тестов прошли на сервере.
- Реальная RTX-камера: fx=528.4337158, RGB 640×480, центры трёх цветных мишеней
  соответствуют проекции, depth=2.41999865 m.
- Оба облака 1920 лучей: конечных 1347/1330, p99 ошибки до комнаты
  0.0203358/0.0207937 m. Сшивка 2230 точек в base_link.
- Движение вперёд 0.386667 вместо 0.4 m, вбок 0.295 вместо 0.3 m,
  yaw 0.58 вместо 0.6 rad: в допуске. Движение после поворота и сдвиг картинки прошли.
- Watchdog остановил базу. Проверка стены не завершилась: 8 s sim time
  не уложились в 120 s wall time, RTF около 0.06 из-за Python-raycast на каждый луч.
- Это не окончательная приёмка. Сохранены runtime/evidence/bridge-verification-scalar.json
  и .log (пути относительно isaac/).

### Текущий дефект и следующий конкретный шаг

Для ускорения написан physx_lidars.py на native PhysX Generic, 1920 лучей за
physics tick, sampling_rate=1920*60, публикация 10 Hz sim time. Вместо отдельных
Python scene queries используется пакетный native API. Первый полный прогон
прошёл камеру, но lidar p99=0.5837 m: обнаружена ошибка знака вертикального угла.
Доказательство сохранено isaac/runtime/evidence/bridge-verification-native-first.json.

Короткая отдельная диагностика isaac/runtime/physx_pattern_probe.py уже выполнена
на сервере без полной модели/камеры (около 13 секунд, exit=0). Результат
isaac/runtime/native-pattern.npz; лог runtime/evidence/native-pattern.log.
Native API возвращает для azimuth=0, zenith=-0.3 точку с ПОЛОЖИТЕЛЬНЫМ z:
local [4.417265, 0, 1.3664203]. Значит SDK-формула z=-sin(zenith),
тогда как исходный Gazebo elevation положителен вверх. Азимут и порядок верные.
Сейчас pattern ошибочно использует +arcsin(beams[:,2]); заменить на МИНУС arcsin.
Не менять порядок лучей и не искать фазовый сдвиг: native azimuth начинается
0, .0528, .1056; 1920 лучей получаются точно. Дальше:

1. Вынести адаптацию native-углов в bridge_core и добавить регрессию знака.
2. Повторить короткую диагностику; сравнить native point_cloud с canonical beams*depth,
   затем проверить соответствие точек стенам/полу. Не запускать полную сцену до успеха.
3. При старте очищать старый ready.json (сейчас он может оставаться от старого запуска).
4. Копировать исправления на сервер, запустить bash isaac/start_bridge.sh.
   Он пересоздаёт только собственные noise/stitching; собственный stopped sim удаляет.
   Полный старт может занять 120 s: сначала RTX engine, затем настоящий RGB/depth.
5. Из al-isaac-navigation запустить verify_bridge.py. Приёмка по внешним DDS,
   result.json в isaac/runtime/navigation/bridge-verification. Требуется status=passed
   и AL_EXTERNAL_BRIDGE_PASSED; все прежние критерии, стена, TF/частоты, reset.
   В тест недавно добавлены лидары после поворота/сдвига и сверка actual USD
   Model/torso world transform с manifest (допуск 3e-5), они ещё не прошли полный тест.
6. Измерить RTF после оптимизации; не выдавать sim-time Hz за wall-time Hz.
   Default viewport updates уже отключаются после warmup, sensor render остаётся активен.
7. Запросить /isaac/shutdown, дождаться остановки и проверить result.json/exit/OOM.
8. Сохранить доказательства, завершить report/review, обновить TASK_REGISTRY,
   iterations/README и инструкции. При необходимости добавить helper остановки/reset.
   В test_navigation.sh включить новые core-тесты; один раз прогнать нужные проверки.
9. Коммит на сервере в isaac, синхронизация ноутбука и push origin isaac с ноутбука.
   Git C:/Program Files/Git/cmd/git.exe; remote workstation указывает на сервер.
   Перед ff merge локальные идентичные изменения нужно сохранить корректно,
   не применять force/reset и не затирать работу. Runtime/evidence не коммитить.
10. Начать перенос СОБСТВЕННОГО мира со стеллажами, HD15/HD17 и реальной STag-детекцией.
    Это следующий этап после объединённых движения/сенсоров, номер в реестре уточнить.

Текущее состояние контейнеров проверено при остановке: al-isaac-astribot НЕ работает;
al-isaac-navigation, al-isaac-odom-noise, al-isaac-lidar-stitching работают.
ros-humble-container и isaaclab_astribot_container — команды, НЕ менять.
В коде runtime общий reset: /isaac/reset, затем /isaac/reset_noise; clock монотонный.
Полная модель — визуальная фиксированная поза, planar collision guard покрывает
только зарегистрированные четыре стены; произвольные mesh-объекты пока не покрыты.
Это надо учесть до принятия собственного магазинного мира.

## Продолжение 09.11.2026 после 06:17 МСК

Знак native zenith исправлен отдельным physx_generic_angles. Шестая регрессия
проверяет native convention по всем исходным лучам. Короткая нативная диагностика
после исправления: max разница reconstructed/native point cloud 1.37e-6/1.43e-6 m,
p99 соответствия исходным поверхностям 5.71e-7/6.94e-7 m, конечных 1347/1330.
Шесть core-тестов прошли (0.18 s).

Полный первый исправленный native-прогон прошёл геометрию обоих лидаров в покое
и после движения, камеру/depth, три оси и повёрнутый ход, watchdog, стену
(x=3.67 m), reset и TF. Итог всё равно failed: внешние RGB/depth получены
лишь около 18 Hz sim-time вместо 30 Hz из-за потерь крупных UDP samples.
Сохранены evidence/bridge-verification-native-udp-drop.json,
bridge-verification-native-fixed.log и bridge-native-udp-drop-runtime.json.
Численные допуски не смягчались.

Добавлен собственный Fast DDS XML: UDPv4 send/receive socket buffers 4 MiB
на обоих концах, без изменений host sysctl. На сервере rmem_max/wmem_max уже 4 MiB.
У STag обнаружено обязательное reliable CameraInfo при best-effort image;
publisher калибровки исправлен на reliable. Готовность/внешний result очищаются
перед новым запуском, чтобы старый JSON не выдавался за текущий успех.

Профилирование следующего запуска обнаружило основной тормоз: ~227.8 ms
за physics tick в публикации камеры при всего ~7.6 ms physics+render.
RTF ~0.069. Нативные лидары сами по себе этот тормоз не устранили.
Причина проверена в сгенерированном ROS Image.data setter: для bytes он делает
два Python-прохода по каждому байту при включённых assertions. Использован его
публичный fast path array.array('B') для RGB/depth/cloud, без изменения wire data
и без отключения проверки типов глобально. Этот диагностический прогон остановлен
намеренно, а не выдан за пройденную приёмку: evidence/bridge-profile-before-array.json
и bridge-verification-profile-interrupted.json/.log.

Общие проверки: 35 C++ tests без failures/errors; 175 Python passed, 1 skipped
(включая 6 новых core-тестов), evidence/test-navigation-005.log.
Две новые проверки исходного мира относятся к 006 и прошли отдельно.
Итоговый живой прогон окончательного кода ещё выполняется; результат добавить ниже.

Документация native API и транспорта:
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_generic.html
- https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html

## Итоговая приёмка окончательного кода

09.11.2026 около 06:35 МСК verify_bridge.sh в отдельном ROS-контейнере завершился
с exit=0, AL_EXTERNAL_BRIDGE_PASSED и status=passed в
isaac/runtime/navigation/bridge-verification/result.json. Рядом torso_calibration.png.

| Проверка | Фактический результат |
|---|---|
| CameraInfo/кадр | reliable calibration, fx=528.4337158, RGB 640×480 optical; глубина 2.41999888 m |
| Проекции мишеней | red [221.5,239.5], green [417.5,239.5], blue [319.5,141.5]; ошибки <1 px |
| Front/back | 1347/1330 конечных из 1920; p99 .0203361/.0207941 m |
| После движения | lidar p99 .0224695/.0204040 m; red сдвинулся до x=568 px |
| Сшивка | 2232 точки, frame base_link |
| Вперёд 2 s × .2 | .3966667 вместо .4000000 m |
| Вбок 2 s × .15 | .3000000 m |
| Поворот 2 s × .3 | .6000000 rad |
| Ход при повороте | delta [.1590171,.1099607] m, в допуске |
| Watchdog | остановился на x=.1 m, затем неподвижен |
| Стена | остановка x=3.67 m при пределе 3.675 m |
| Reset | GT и noisy pose [0,0,0], clock монотонный |
| TF | 38 static edges, только odom→base_link динамический, owner odom_noise_node |
| Частоты sim time | GT 50.0323, RGB/depth 30.0242, оба lidar и joined 10.0081 Hz |
| Свежесть | RGB/depth stamps строго возрастают, остальные не идут назад |
| Фактический USD | max error Model/torso относительно base@manifest 3.67322e-6 |
| Скорость исполнения | RTF=0.99315 во внешнем тесте |

После теста отдельно проверены CLI reset и штатный shutdown. Контейнер завершился
exit=0, OOM=false; JSON status=finished. Весь финальный runtime 42.766669 s sim /
43.030348 s wall; сообщений odom=2139, RGB/depth=1284, front/back=428.
Средняя стоимость на physics tick: physics/render 7.399 ms, camera 1.667 ms,
lidar .143 ms, base .467 ms, odom .423 ms. Устранение setter bottleneck
снизило camera-участок с ~228 ms до ~1.67 ms; realtime pacing ограничивает RTF около 1.

Логи/JSON: isaac/runtime/evidence/bridge-verification-acceptance.log,
bridge-accepted-runtime.json/.log, test-navigation-005.log.
Результаты и кадры скопированы на ноутбук; тяжёлые артефакты в Git не входят.
Собственные noise/stitching остановлены, navigation оставлен готовым; команда не затронута.
Код и отчёты публикуются в isaac. Следующая итерация 006 уже начата аудитом мира.

Принимаются именно движение и штатные feeds в комнате. Полная магазинная сцена,
STag-детекции, Nav2, произвольные mesh-collisions и длительная стабильность этим
прогоном не доказаны. Фиксированные поза тела и planar base сохраняют выбранный
кинематический подход; колесная контактная динамика не добавлена.
