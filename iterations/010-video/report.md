# Итерация 010 — короткое видео эксперимента

09.11.2026, отдельная просьба пользователя быстро записать простой ролик.
Новый демонстрационный run `demo-010-active`, active/seed42/noise×4,
исходный shelves_south_wall. Это не повтор для замены результатов серии009.

`isaac/record_demo.py` записывает настоящий ROS torso RGB в H264 MP4,1280×720/15fps.
Справа — собственная occupancy-карта и GT-траектория/направление как явно подписанный
диагностический overlay. GT не подаётся в навигацию. Запись начинается при первом
движении базы, так что прогрев и ожидание инициализации в ролик не попадают.
Параметры эксперимента и алгоритмы не изменяются; дополнительная запись влияет
на нагрузку, поэтому этот run не используется в benchmark statistics.

Артефакт: `isaac/runtime/navigation/videos/experiment-active-010.mp4`,
метаданные рядом `.json`. Исходные Nav2 result/acceptance/container logs —
`runtime/navigation/experiments/demo-010-active/`. Runtime-видео вне Git.

Готово:353 кадра,15fps,23.533s,H264,1280×720,1,319,923bytes.
Декодированы все353 кадра; camera frames меняются, средний кадр просмотрен.
Две Nav2 goals succeeded, обе pose acceptance passed; recorder/video и остальные
собственные процессы exit0/OOM=false. Проверен новый recorder adapter на живом demo.
MP4/metadata скопированы на ноутбук; SHA256:
`d86935cb6f08bd509e8ca4ce96688a04230e21872f22e286d245b6416619ac3a`.
