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

> **T-011c:** by default `run_evo.sh` first crops all series to the
> evaluation window (goal 1 start .. last goal end, from `result.yaml`) and
> also runs evo with `-r angle_deg` (`ape_rot.zip`, `rpe_rot.zip`); the
> commands below are run on `gt_eval.tum` / `est_eval.tum`. See §7.

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
* detection coverage is measured over the evaluation window (§7; the GT
  time span if there is none).
* map→odom step = largest change between consecutive samples; correction
  = largest |map→odom| (`--map-odom-ref zero`, default since T-011c) or
  |map→odom − first sample in the window| (`--map-odom-ref first`). Both
  are properties of the transform, not of the robot pose jump.
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

Tests (no ROS; evo is mocked, plus end-to-end tests with the real evo
that are skipped if `evo_ape` is missing):
`python3 -m pytest -q eval navigation/src/al_eval/test`.

## 7. Evaluation window, rotation, paired view, sanity flag — T-011c

Implements the requests of the RND analysis plan (`04_analysis_plan.md` §6).
Defaults changed in this version: **cropping on**, **map→odom reference
`zero`** (was `first`).

**Evaluation window** (`eval/crop_run.py RUN_DIR`). The recording contains
the start-up (localization init, relocalization jump of up to ~1 m, the
0.6 m nudge) and the time after the last goal. Everything is evaluated on
`[t0, t1]` = [`start_sim_s` of the first goal, `end_sim_s` of the last
goal], new per-goal fields of `result.yaml` written by `scenario_runner`
(sim time of goal sending / goal end, the same clock as the TUM stamps when
the runner runs with `use_sim_time:=true` — *[unverified on the testbed]*).
`crop_run.py` writes `gt_eval.tum` (GT padded by `--gt-pad` 0.1 s on both
sides so that evo can interpolate the est poses at the edges),
`est_eval.tum`, `map_odom_eval.tum`, `landmarks_eval.csv` and
`eval_window.yaml` (t0, t1, source, row counts). Without the stamps (old
`result.yaml`, no `result.yaml`, end ≤ start, or a window that does not
overlap the GT span — wall vs sim clock) it falls back to the full GT range
and prints a `WARNING`; `--window T0 T1` sets it by hand.

* `run_evo.sh` crops by default and evaluates the `*_eval` files into
  `evo_<align>/` (+ a copy of `eval_window.yaml`); `--no-crop` evaluates the
  whole recording into `evo_<align>_full/` (never mixed with the cropped
  results). On the legacy `shift/noise/drift` cases (no `result.yaml`) the
  numbers are unchanged (APE 0.1000 / RPE 0.0000 for `shift`).
* `summarize.py` and `plots.py` apply the same window in memory (same
  functions) to GT, landmarks and map→odom, and read `evo_<align>/`
  (`--no-crop`: whole recording, `evo_<align>_full/`). `summarize.py`
  notes a run whose `evo_<align>/eval_window.yaml` is missing (results of
  the old `run_evo.sh`: rerun it) or differs from its own window.

**Rotation.** `run_evo.sh` also runs `evo_ape/evo_rpe -r angle_deg` with the
same association, alignment and RPE delta (`ape_rot.zip`, `rpe_rot.zip`,
logs, a `SUMMARY_ROT` line). Columns `ape_rot_rmse`, `ape_rot_max`,
`rpe_rot_rmse`, `rpe_rot_max` [deg].

**New per-run columns** in `summary.csv` / `summary.md` (all on the window):

| column | definition |
|---|---|
| `rot_total_rad` | Σ\|wrap(Δψ)\| of the GT yaw [rad] (D-3 injects yaw noise per radian turned) |
| `mean_speed_mps` | GT path length / duration |
| `det_max_gap_s` | longest interval of the window without a detection (edges included) [s] |
| `mo_init_m`, `mo_init_yaw_deg` | \|map→odom\| of the *first sample of the whole recording* |
| `sanity_flag` | `bad_db_init` if that sample is > `--init-tol-m` 0.05 m or > `--init-tol-deg` 2° from the identity, else `ok`; empty without `map_odom.tum` |
| `seed`, `window_source`, `window_t0/t1`, `window_s` | pairing key; `result.yaml` / `full range` / `off (whole recording)` |

Detections per m, coverage, map→odom max step / correction and path length
are now computed on the window as well. Flagged runs **stay in the
statistics** (not silently dropped); they are listed at the top of
`summary.md` and in a comment of `summary_table.tex`.

**`summary_paired.csv`**: one row per seed (`seed` from `run.yaml`, else
`result.yaml`, else the run index `runNN`), the run of each mode, flagged
runs, and for the main metrics (`ape_rmse ape_max rpe_rmse ape_rot_rmse
det_per_m det_coverage_pct det_max_gap_s mo_max_corr_m duration_s
path_length_m rot_total_rad`) the baseline value and `d_<mode>_<metric>` =
mode − baseline. `summary.md` adds a table with the mean paired difference,
the sign consistency `k/n` and the baseline spread (max − min) to compare
control − baseline against.

**`summary_table.tex`** (report Table 1, T-012b): rows = APE RMSE [m], APE
max [m], detections per m, max map→odom correction [m], time [s], path
length [m], goals reached (sum k/N); columns = modes; mean ± sample std. It
sets `\tabcolsep` to 3 pt and fits one column of the report (checked with
`report/build.sh`: no overfull box, 2 pages). The full metric set stays in
`summary.md` / `summary.csv`.

**Plots.** `trajectories`: est heading arrows every `--arrow-spacing` m of
est path (default 0.5, 0 = off). `ape_vs_time`: below APE = 0 one rug row
per run with a tick at each detection (landmarks.csv, n ≥ 1), on the APE
time axis (seconds from the first evaluated pose).

**Synthetic data.** `make_synthetic.py --case experiment` now writes the
per-goal stamps, a 4 s pre-roll (est has a 0.3 m init transient that the
window must remove: APE max 0.31 m whole recording vs 0.06 m cropped on
`control_03`) and a 2 s post-roll; `control_03` has a wrong DB init
(map→odom = 1 m / 157° for 1.5 s) and is flagged.
