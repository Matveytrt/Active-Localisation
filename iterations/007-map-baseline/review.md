# Саморевью 007 — карта и baseline pilot приняты

Самопроверка ассистента, независимый проверяющий не участвовал.
Проверить входы SLAM, allowlist/age gate, единственных TF owners, новую DB и
её provenance, optimized export, отдельный localization reset и реальные
NavigateToPose результаты/фактические цели. До внешней приёмки этап не принят.

Итог: внешние mapping-route, optimized-map evaluation, реальный Nav2 action probe
и graph audit passed. Ненулевой HD17 ID1 проверен (9.27 mm), stream только 1–5,
пустой взгляд прекращает новые landmarks после выдержки. Exact .3 s boundary
не измерена, это сохранённая исходная настройка, а не отдельная новая метрология.

Прямой exporter bounded WM дал лишь один cached landmark: это не приняли и
не заменили raw pose fallback. Native RTAB-Map global optimization включил LTM,
вернул 5 landmarks; hashes Data/Link/raw Node poses до/после совпали. Source/GT
не читались finalizer и не поступали в SLAM. Median optimized XY error 1.74 cm.

Свежий Isaac/ROS localization reset, отдельная DB-копия и известный initial seed;
pristine hash неизменен. Три настоящих NavigateToPose SUCCEEDED, max estimated
XY 8.97 cm, max GT XY 8.93 cm, yaw в допуске. Ручного cmd_vel во время целей нет.
TF/map/odom owners и допустимые publishers проверены отдельным observer.

Failed первый OOM-прогон, first lifecycle-race rejected goal и SIGTERM shutdown
не скрыты. Bounded WM/cache и общий velocity override явно отличаются от исходных
unlimited/velocity defaults; сравнения baseline/active/control обязаны делить этот профиль.
После исправления памяти mapping/localization simulator и RTAB-Map exit0/OOM=false;
Nav2 cleanup SIGINT отдельно exit0/OOM=false. 35 C++ +179 Python passed, 1 skip.

Решение: принять 007 как собственную карту и один baseline pilot. Не называть
это воспроизведением всего active-localization эксперимента или статистическим
преимуществом над Gazebo/active/control. Следующий шаг 008 — единый runner и режимы.
