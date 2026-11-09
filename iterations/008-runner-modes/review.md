# Саморевью 008 — принято в пределах paired pilot

Самопроверка ассистента, независимый reviewer не участвовал. 09.11.2026.

Сверены configs: baseline v2, active active; control ровно три prescribed
изменения, landmarks из optimized Isaac DB, общий bounded SLAM/velocity profile.
Оригинальные algorithm packages не изменены. Logs/ROS params подтверждают
активные planner/layer, 5 markers и null/disabled control.

Равные pristine/scenario/landmark hashes, start/seeds, отдельные writable DB и
полные restart зафиксированы manifest. Все 6 NavigateToPose goals succeeded;
estimated и GT errors в допусках. Raw GT/est/noisy/map_odom и /plan/feedback
сохранены, TUM/clock audit passed. Active/control effect .4439m/.95185rad
превышает пороги до runs. Active: 30 accepted HD17 detections IDs1/2, остальные0.

Первый failed shutdown сохранён. Исправлены process wrappers/rclpy shutdown,
sender изолирован. Финально все 10 контейнеров каждого эпизода exit0/OOM=false;
чужие контейнеры не тронуты, pristine не менялась. 35 C++ +190 Python passed/1skip,
отдельно9 seed CLI passed, live lock guard passed.

Границы: один pilot seed42, MPPI RNG uncontrolled, порядок baseline/active/control,
startup/ROS scheduling не детерминированы. Runs имеют uncommitted source hashes;
shell leading-zero parser уточнён после серии и проверен целевыми tests.
Статистическое преимущество не доказано; active медленнее baseline по wall time.
APE/RPE ещё не вычислены. Fault injection timeout активной Nav2 цели не было.
PnP off-axis/raw false positives, кинематическая база и длительная надёжность
остаются ограничениями. Privacy fix очищает текущие файлы, не старые Git commits.

Решение: принять 008 как работающие режимы и общий runner с одним paired pilot.
Перейти к 009 после отдельного запуска; не объявлять весь эксперимент воспроизведённым.
