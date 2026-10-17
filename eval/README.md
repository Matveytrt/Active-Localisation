# eval — APE/RPE через evo

Проверено с **evo 1.38.0** (требует Python ≥ 3.10, matplotlib ≥ 3.6).

```bash
pip install evo            # или: pip install --user evo / в venv
# В Ubuntu 22.04 системный matplotlib 3.5 — pip поставит новее в user/venv;
# если мешает ROS-инструментам (rqt_plot), ставьте evo в отдельный venv.
```

## 1. Записать прогон

```bash
ros2 launch al_eval traj_recorder.launch.py run_name:=baseline_01 output_dir:=~/al_runs
# прогон сценария ... Ctrl+C
ls ~/al_runs/baseline_01   # gt.tum est.tum meta.yaml
```

Проверить `meta.yaml`: `counters.est_written` > 0, доля `est_skipped_*`
небольшая, `gt_written` ≈ 50 × длительность. Про кадр GT и `gt_offset_*` — см.
`navigation/src/al_eval/README.md` (кадр `/odom_gt` пока **не проверен**).

## 2. Посчитать метрики одного прогона

```bash
eval/run_evo.sh ~/al_runs/baseline_01                 # без выравнивания (по умолчанию)
eval/run_evo.sh ~/al_runs/baseline_01 --align se3     # Umeyama SE(3)
```

Результаты: `<run>/evo_<align>/` — `ape.zip`, `rpe.zip`, графики
`ape_raw.png ape_map.png rpe_raw.png rpe_map.png` (или `--plot_format pdf` →
`ape.pdf`, `rpe.pdf`), логи `ape.log rpe.log`, `commands.txt` (точные команды).

Фактически выполняемые команды (без выравнивания):

```bash
evo_ape tum gt.tum est.tum -r trans_part \
    --sync_method interpolation --t_max_diff 0.02 --no_warnings \
    --project_to_plane xy --save_results evo_none/ape.zip \
    --plot_mode xy --save_plot evo_none/ape.png
evo_rpe tum gt.tum est.tum -r trans_part --delta 1 --delta_unit m \
    --pairs_from_reference \
    --sync_method interpolation --t_max_diff 0.02 --no_warnings \
    --project_to_plane xy --save_results evo_none/rpe.zip \
    --plot_mode xy --save_plot evo_none/rpe.png
```

Почему так:

* `-r trans_part` — ошибка по трансляции, м (в evo нет значения
  `translation`).
* Ассоциация по времени: GT 50 Гц, est 10 Гц. `--sync_method interpolation`
  интерполирует GT в моменты est (после сопоставления в пределах
  `--t_max_diff`, по умолчанию 0.02 с — шаг GT). С `nearest_time` берётся
  ближайший отсчёт GT, и в ошибку попадает до ~10 мс × скорость.
* `--project_to_plane xy` — робот плоский; z GT (высота спавна 0.16) и z
  `map→base_link` могут отличаться, без проекции разница по z войдёт в APE
  (на синтетике: 0.100 → 0.189 м). Отключить: `--3d`.
* RPE `--delta 1 --delta_unit m --pairs_from_reference`: пары поз через 1 м
  пути, путь меряется по GT (шум est не удлиняет путь). Для 12 м пути это
  всего ~12 пар — RPE шумная; при необходимости `evo_rpe ... --all_pairs`.
* Графики без GUI: evo **игнорирует `MPLBACKEND`** и берёт backend из
  `~/.evo/settings.json`. Поэтому скрипт по умолчанию запускает evo с
  временным `HOME` (backend Agg, пользовательские настройки не трогаются;
  `PYTHONUSERBASE`/`MPLCONFIGDIR` сохраняются). `--user_settings` — запуск с
  вашими настройками. Метрики от этого не зависят.

## 3. Выравнивание — что именно измеряем

| `--align` | что делает | что поглощает |
|---|---|---|
| `none` (по умолчанию) | ничего | ничего: постоянный сдвиг GT↔map **входит** в APE |
| `se3` | evo `-a`, Umeyama SE(3) по всем позам | любой постоянный жёсткий сдвиг/поворот, а также часть дрейфа |
| `origin` | evo `--align_origin`, первая поза est → первая поза GT | начальный сдвиг; ошибка в первой позе переносится на всю траекторию |
| `posyaw` | `align_posyaw.py`: 2D Umeyama (yaw + xy), затем evo без выравнивания | как `se3`, но строго 4-DoF; печатает оценку `gt_offset_*` |

**Важно для отчёта.** Без выравнивания APE содержит постоянный сдвиг между
кадром GT и `map` (неверный `gt_offset` даст ошибку, не связанную с
локализацией). С `se3`/Umeyama этот сдвиг поглощается — и вместе с ним
поглощается реальная постоянная ошибка локализации, а при дрейфе APE
занижается (на синтетике с дрейфом: 0.173 → 0.072 м). В отчёте нужно явно
писать, какое выравнивание использовано, и сравнивать baseline/active только
при одинаковом выравнивании (`aggregate.py` это проверяет). RPE к постоянному
сдвигу нечувствительна (0 на синтетике со сдвигом).

## 4. Агрегировать прогоны

```bash
python3 eval/aggregate.py \
  --runs ~/al_runs/baseline_0{1,2,3} ~/al_runs/active_0{1,2,3} \
  --labels baseline baseline baseline active active active \
  --align none --out results/
```

`--runs` — каталоги прогонов (берётся `evo_<align>/`) или сразу каталоги с
`ape.zip`/`rpe.zip`. Выход: `results/results.csv`
(`run,label,ape_rmse,ape_mean,ape_max,rpe_rmse,rpe_mean,align,ape_n,rpe_n`),
`results/summary.md` (mean ± std по меткам, std выборочное),
`results/ape_rmse_boxplot.png` (box plot APE RMSE и RPE RMSE по меткам).
Статистика читается прямо из `stats.json` внутри zip (то же, что
`evo.tools.file_interface.load_res_file`). Смешение разных выравниваний —
ошибка (обход: `--allow_mixed_alignment`).

Штатная альтернатива evo: `evo_res ~/al_runs/*/evo_none/ape.zip
--use_filenames --save_table table.csv` (каталог для таблицы должен существовать).

## 5. Самопроверка на синтетике

```bash
python3 eval/make_synthetic.py /tmp/syn           # shift / noise / drift
eval/run_evo.sh /tmp/syn/shift                    # APE rmse 0.1000, RPE 0.0000
eval/run_evo.sh /tmp/syn/shift --align se3        # APE rmse 0.0000
```

GT — окружность r = 2 м, 60 с, 50 Гц; est 10 Гц со сдвигом меток +4 мс:
`shift` = +0.10 м по x; `noise` = N(0, σ²) по x и y (APE RMSE ≈ σ√2);
`drift` = +5 мм/с по x. Отдельный прогон:
`make_synthetic.py OUT --case noise --sigma 0.02 --seed 7 --name active_1`.

## 6. Experiment summary and plots (3 modes, secondary metrics) — T-011a

Run dirs `<mode>_<NN>` for the modes `baseline`, `active`, `control` (D-2),
each processed by `run_evo.sh` first. Besides `gt.tum` / `est.tum` the
recorder now also writes (see `navigation/src/al_eval/README.md`):

| file | content |
|---|---|
| `landmarks.csv` | `stamp,n_detections,ids`, one row per `/landmark_detections` message (`rtabmap_msgs/LandmarkDetections` from `tf_to_fiducial`; ids `;`-separated) |
| `map_odom.tum` | TF `map -> odom` (the RTAB-Map correction) at `rate_hz`, TUM, duplicate stamps skipped |

```bash
for r in ~/al_runs/*_0?; do eval/run_evo.sh "$r"; done
python3 eval/summarize.py ~/al_runs --out results/            # tables
python3 eval/plots.py ~/al_runs --out results/plots --score-map  # figures
```

`summarize.py RUNS_ROOT` (or `--runs DIR ...`) groups runs by `mode` in
`run.yaml` (fallback: the dir name prefix before `_<NN>`) and writes:

| file | content |
|---|---|
| `summary.csv` | one row per run: APE rmse/mean/max, RPE rmse, duration, GT path length, goals succeeded/total, detections (total, per m, % of time with ≥ 1 detection in the last `--window` s, default 1 s), map→odom max step and max correction (m, deg), notes on missing inputs |
| `summary_modes.csv` | per mode: `<metric>_{mean,std,min,max}` (std = sample std, ddof 1) |
| `summary.md` | mode table (mean ± std), [min, max] table, change vs baseline, per-run table |
| `summary_table.tex` | compact booktabs `tabular` for the report (`\usepackage{booktabs}`, then `\input{summary_table.tex}`) |

Definitions:

* duration = sum of `duration_sim_s` of the goals in `result.yaml`
  (fallback: time span of `gt.tum`, noted in `duration_source`);
  path length = integrated xy distance of `gt.tum`.
* detection coverage is measured over the GT time span `[t_gt0, t_gt1]`.
* map→odom step = largest change between consecutive samples; correction
  = largest |map→odom − first sample| (`--map-odom-ref first`, removes the
  constant spawn offset) or |map→odom| (`--map-odom-ref zero`). Both are
  properties of the transform, not of the robot pose jump.
* Missing `evo_<align>/`, `result.yaml`, `landmarks.csv` or
  `map_odom.tum` give empty cells plus a warning. Different alignments
  across runs are an error (as in `aggregate.py`; `--allow_mixed_alignment`).
* `aggregate.py` is unchanged and still works for APE/RPE only.

`plots.py RUNS_ROOT` writes PNG + PDF into `--out` (default `ROOT/plots`):
`trajectories` (one panel per mode, GT thin dashed, est thick, colour =
run index), `ape_vs_time` (all runs + mean over runs, from
`evo_<align>/ape.zip`), `detections_path` (GT position at each detection,
coloured by `n_detections`). Background: `--score-map` (S2* s* map computed
with `navigation/src/active_localization/tools`) or `--background
FILE.png|FILE.npy --extent XMIN XMAX YMIN YMAX` (PNG row 0 = top; `.npy`
row 0 = `ymin`, the `score_maps.py` convention).

Self-check on synthetic data (3 modes × 3 runs, all files of a real run):

```bash
python3 eval/make_synthetic.py /tmp/exp --case experiment   # [--runs N --seed S]
for r in /tmp/exp/*_0?; do eval/run_evo.sh "$r" --plot_format none; done
python3 eval/summarize.py /tmp/exp && python3 eval/plots.py /tmp/exp --score-map
```

The synthetic model (see the `make_synthetic.py` docstring) is a toy:
active sees landmarks on more of the lap, so its APE is lower and its
detections per metre higher; control behaves like baseline.

Tests (no ROS; evo is mocked, plus one end-to-end test with the real evo
that is skipped if `evo_ape` is missing):
`python3 -m pytest -q eval navigation/src/al_eval/test`.
