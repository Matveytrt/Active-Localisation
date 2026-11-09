# Итерация 009 — воспроизведение протокола и оценка

09.11.2026. Статус: завершена как воспроизведение протокола с явно сохранёнными
исключениями приёмки: Nav2 90/90, строгая поза89/90, runner28/30.
Начальная ветка isaac, commit 6e00a2a;
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
goal, startup исключён. GT используется исходной моделью шума и для evaluation,
но не передаётся напрямую в SLAM/Nav2. Source crop/summarize/aggregate не менять.
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

## Исполнение и неизменность условий

Ядро и протокол зафиксированы до начала commit `44d02d036be09f6a16e9362b3dab15bf07639964`.
Серия `reproduction-009` выполнила все30 jobs. Во время серии менялся только реестр;
runtime/algorithm sources и HEAD не менялись (проверено по manifest каждого run).
Ротация режимов циклическая; при5 seeds она не даёт абсолютно равного числа позиций.
Одинаковые noise/lidar seeds не означают одинакового дрейфа на разных траекториях;
RNG MPPI не управляется. Никаких selective reruns или tuning по результатам не было.

Каждый эпизод: отдельная pristine DB-копия, новые Isaac/ROS/noise/STag/SLAM/Nav2,
одинаковый spawn и один fixed initialpose, исходные3 NavigateToPose goals.
Runtime noise coefficients проверены через ROS services: variance×k², bias×k.
Generated Nav2 YAML сравнен с ожидаемым исходным профилем; controller/planner/layer
params проверены в живом графе. Active действительно загрузил5 markers и вычислял
cost layer. Null control ожидаемо сообщает0 markers в scorer и disabled layer;
при этом RTAB-Map и реальные детекции HD17 работают одинаково во всех режимах.

Canonical map SHA до/после каждого run:
`70a236237a60e4c4294d31ab308f449bdaa1e5be5dab5399742701a8ed6397fe`.
Landmarks/scenario SHA также совпали. Оба native lidars/RGB/depth/odom реально
публиковались; world/spawn/seeds и все10 собственных контейнеров проаудированы.
Сторонние контейнеры не изменялись; после работ остался только свой idle navigation.

## Навигационные исходы и исключения

Все90 actions имеют статус succeeded. Все30 эпизодов включены в метрики.
Строгий дополнительный gate XY≤.15m/yaw≤.10rad/GT XY≤.30m прошли89/90 целей:
control×6 seed44 goal0 дал estimated yaw error **.1665058rad**, хотя Nav2 вернул success.
GT XY всех90 целей ≤.136943m. Сохранены исходные result/acceptance и raw /plan;
последний pose плана первой цели сохранял точный requested yaw1.5708. Причина
расхождения snapshot pose с моментом внутреннего goal check Nav2 отдельно не установлена.
Допуски и алгоритмы не менялись, этот случай не исключался из статистики.

Другой failed runner: control×4 seed45. Все3 goals и pose gates прошли, но recorder
получил RuntimeError в native take_message при SIGINT, exit1/OOM=false. Meta финализирована
после конца окна (49.816669s против goal end49.583336s); GT2486/est494/map_odom495
и29 landmark messages совпадают с файлами. Запись покрывает окно; её данные не потеряны.
Остальные9 контейнеров этого run exit0/OOM=false. Все другие runs без OOM/kill.

Поэтому **runner28/30**, **action90/90**, **strict pose89/90**, **teardown29/30** —
разные проверенные итоги. Исходный failed verdict и summary до уточнения classification
сохранены. Проверка input/data теперь отделена от teardown: данные и входы30/30 valid,
но teardown failure1 остаётся в итоговом JSON/CSV. Это не выдаётся за30 зелёных эпизодов.

После завершения серии добавлен `run_recorder.py`: использует исходный TrajRecorder,
отключает native signal handlers и завершает spin_once до finalize/destroy/shutdown.
Неожиданные ошибки не подавляются. Отдельная synthetic ROS fixture проверила20/20
последовательных SIGINT при входящих clock/odom/TF, exit0 и совпадение счётчиков/файлов.
Основная серия не повторялась после исправления; её статистика остаётся первоначальной.

## Оценка траекторий

Исходные crop_run/run_evo/summarize не менялись. Изолирован eval venv:
evo1.38.0, numpy1.26.4, scipy1.15.3, matplotlib3.10.7, PyYAML6.0.3.
Первый shared-system venv дал ABI error SciPy/NumPy **до основной серии**;
ошибка сохранена, исправлена полной изоляцией. Системный ROS Python не заменён venv.

30/30 raw TUM finite, unit quaternions, строго возрастающие timestamps;
счётчики meta соответствуют файлам, запись покрывает конец окна целей.
Сопоставлено100% cropped est samples:348–545 поз на run; RPE15–16 пар через1m.
Data/input audit failures0. Доля пути с недавней детекцией интегрируется по длине,
отдельно от временной coverage исходного summarize. Аналитические тесты различают
наблюдение при стоянке и при движении.

Все30 первых raw map→odom получили диагностический bad_db_init (сохранённая pose
из карты: .054m/1.5° до known initialpose). Флаг не удалялся и runs не отбрасывались.
После fixed seed max initial estimate error .001331m/.000323rad, до начала целей.
Так отделена сохранённая стартовая TF от реально принятой инициализации.

| Шум | Режим | APE RMSE mm | RPE RMSE mm | Время sim s | Det/m | Путь с det за1s % |
|---|---|---|---|---|---|---|
| ×4 | baseline | 37.96±1.71 | 59.30±9.45 | 36.83±1.61 | 1.41±.35 | 17.56±1.70 |
| ×4 | active | 34.42±.90 | 62.88±4.74 | 48.73±2.36 | 11.54±.95 | 53.94±2.39 |
| ×4 | control | 36.12±2.07 | 45.85±2.60 | 42.15±1.71 | 1.85±.36 | 15.84±2.22 |
| ×6 | baseline | 53.52±4.69 | 85.49±11.67 | 40.91±4.88 | 2.05±.87 | 12.29±4.64 |
| ×6 | active | 47.60±1.14 | 73.77±11.37 | 52.81±2.42 | 11.42±.57 | 53.06±2.81 |
| ×6 | control | 51.31±2.78 | 81.67±13.85 | 45.36±3.32 | 1.35±.82 | 10.42±4.45 |

По5 runs, mean±sample std. Paired active-minus-baseline APE RMSE:
×4 −3.542mm CI95[−6.063,−1.020], ×6 −5.912mm [−11.515,−.309].
Active-minus-control: ×4 −1.702mm [−4.168,.764], ×6 −3.710mm [−7.028,−.392].
Active-minus-control RPE×4 **+17.023mm** [8.646,25.399]; для RPE×6 интервалы
active-minus-baseline/control пересекают0. Это описательные t-интервалы по5 seed,
без поправок на множественные сравнения, не универсальное доказательство метода.

Active повысил coverage пути на36.38/40.76 percentage points против baseline,
добавил11.89/11.90s sim time. Reported Nav2 recovery counts за серию202;
mean/run ×4 baseline/active/control4.4/7.0/1.2, ×6 10.6/12.6/4.6.
Это счётчики feedback Nav2, не внешние повторные эпизоды. Navigation RTF
means .9555/.9656/.9667 для×4 и .9599/.9674/.9665 для×6.
Полные значения/std/CI/pose outcomes — в [evaluation](../../report/isaac/evaluation.md).

## Проверки и финальный отчёт

35 C++ +208 Python passed/1skip до серии. Исходные eval tests31passed/2skip,
затем обе real-evo integration tests отдельно2passed. После добавления path metric
ещё3 analytical unit tests passed. Native identity/translation/z-projection/time
association controls прошли. Recorder shutdown fixture20/20 passed после фикса.
Python compilation, shell/Compose и source/dataset hashes проверены.

Отдельный [итоговый отчёт](../../output/pdf/IsaacReproduction.pdf) —6 страниц о
процессе переноса: модель/движение, камера/лидары/STag, собственный мир/карта,
протокол, трудности и решения, что помогло и что не доказано. Все страницы отрендерены
и просмотрены; иллюстрации — реальные кадры. Editable source:
[reproduction.md](../../report/isaac/reproduction.md). Исходные ActLoc.pdf и report/report.tex
не изменялись; контрольные суммы сохранены/проверяются перед публикацией.

## Артефакты и границы принятия

Remote/local `isaac/runtime/navigation/series/reproduction-009/`:
manifest +logs, evaluation/summary.json/md,runs.csv,paired.png, versions и прежний audit.
`runtime/navigation/experiments/reproduction-009-*`:30 folders с result/acceptance/meta,
raw/eval TUM, исходными evo ZIPs/commands/logs, ROS configs/params/plans/feedback/clock,
bridge/container states. `runtime/navigation/recorder-shutdown-009` и
`evaluation-controls-009-v2` — отдельные synthetic controls, не часть навигационной серии.
Runtime evidence logs находятся под `isaac/runtime/evidence/*009*`.
Mutable DB-копии30 runs остаются на удалённом ПК; canonical DB и все небольшие
исходники/результаты зеркалируются на ноутбук. Runtime не публикуется в Git.

Принята функциональная цепочка воспроизведения исходного протокола в Isaac
с описанными исключениями, а не нулевая ошибка всех gates. Сохранены ограничения
005–008: кинематическая база, off-axis PnP/raw false positives, одна статичная карта,
5 seeds, uncontrolled MPPI, изменённые common execution profile и initialization.
Новая карта/сенсорная реализация/версии не дают трактовать разницу с Gazebo как
эффект только одного фактора. Полная физика и длительный непрерывный runtime не доказаны.
