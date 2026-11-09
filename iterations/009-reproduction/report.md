# Итерация 009 — воспроизведение протокола и оценка

09.11.2026. Статус: в работе. Начальная ветка isaac, commit 6e00a2a;
обе рабочие копии чистые, собственные simulator/ROS остановлены, pristine SHA проверен.

## Протокол до прогонов

По предоставленному двухстраничному отчёту ActLoc.pdf (20.10.2026), основной
сценарий `shelves_loop_a`: три исходные цели, уровни шума ×4/×6, seeds42–46,
baseline/active/control, ротация порядка внутри seed. Итого30 runs/90 goals.
Дисперсионные коэффициенты исходной noise model масштабируются как k²,
систематические scale/yaw drift как k; алгоритмы и пороги не подстраивать по результату.

Используем общий runner008, собственную новую DB/landmarks Isaac, fresh reset,
общие .6/.6/.8 limits/bounded SLAM, один known initialpose. В Gazebo использовалось
начальное перемещение .6m; в Isaac оно заменено принятой инициализацией007.
Это явное различие реализации, не точная физическая/численная копия Gazebo.

APE/RPE: исходный eval pipeline/evo1.38.0, xy, align none, GT interpolated at est
timestamps, max diff .02s, RPE1m/pairs_from_reference. Window первого–последнего
goal, startup исключён. GT only evaluation. Source crop/summarize/aggregate не менять.
Средние/std по5 runs и парные active-baseline/active-control 95% t-интервалы (df4);
интервалы описательные, без обещания значимости и без исключения failed runs.

Критерии принятия: каждый из30 planned jobs учтён (включая failure/timeout),
все90 action outcomes сохранены либо явный skipped/not reached, исходные SHA и
runtime noise profiles проверены, cleanup/pristine invariant, metrics/secondary
metrics/RTF/versions/raw TUM сохранены; analytical evo controls прошли.
90/90 — проверяемый результат, не заранее гарантированное условие.

Исходный Gazebo отчёт не переписывается. После выполнения — отдельный итоговый
документ о процессе воспроизведения: перенос робота/сцены/датчиков/ROS/карты/runner,
технические трудности, что помогло, фактическая валидация, границы эквивалентности.
