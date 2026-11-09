# Итерация 011 — видео с active costmap и статистикой

09.11.2026. По просьбе пользователя расширен demo overlay. Новый отдельный
`demo-011-costmap`, active/seed42/noise×4, shelves_south_wall; серия009 не меняется.

В кадре настоящий torso RGB и **фактическая составная global costmap Nav2**
(static/obstacles/LocalizabilityLayer/inflation). Источник — costmap_raw и GetCostmap,
не раскрашенная static occupancy-карта. Сервис позволяет читать полный актуальный
grid при исходном always_send_full_costmap=false; параметры Nav2 не менялись.
Поверх карты — настоящий /plan с yaw и GT trace, mapped HD17 positions.

Внизу live-статистика из ROS: simulation time, путь GT, accepted landmark detections,
успешные NavigateToPose goals, RTF, detections/m, reported recoveries и live XY RMSE.
Для error GT и map→base сопоставляются по одному timestamp из небольшой истории.
Это online diagnostics с другой частотой выборки, не подмена formal evo statistics009.

Артефакт `isaac/runtime/navigation/videos/experiment-active-costmap-011.mp4`;
metadata рядом. Источники статистики и число costmap reception events сохраняются.
Готово:327frames/15fps/21.8s/H264/1600×900. Декодированы все кадры; средний и
последний просмотрены. Получено36 native costmap messages/service responses,
27 accepted detections IDs1/2, обе Nav2 goals succeeded, путь5.6068m,
live XY RMSE31.19mm по268 time-matched samples. Video exit0/OOM=false.
SHA зеркала `ee40bb7e692bbc9753a5f89338e45e72d19e35c5e524c133b6d500aa2192472c`.

Уточнение пользователю по исходному коду: global costmap update/publish1Hz.
LocalizabilityLayer кеширует максимум видимости по yaw; invalidation происходит
при изменении size/resolution/origin сетки. Движение робота и изменение содержимого
сетки без смены этой геометрии сами по себе не пересчитывают это поле.
В ролике native combined costmap и перепланируемые plan/yaw, а не динамически
обучаемая карта информации. Алгоритм не менялся ради визуализации.
