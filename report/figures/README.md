# report/figures

Figures used by `report/report.tex`. A missing file is drawn as a red TODO box,
so the report always builds.

| File | Status | Produced by | Used in |
|---|---|---|---|
| `s_star.png` | present | `navigation/src/active_localization/tools/score_maps.py --out <dir>` (T-009a; best-over-yaw S2* score, 36 yaws) | Fig. 1 (method) |
| `trajectories.pdf` | TODO (T-011a) | `eval/plots.py`: estimated (thick) and GT (thin) trajectories per mode on top of the s* map | Fig. 3 left |
| `ape_vs_time.pdf` | TODO (T-011a) | `eval/plots.py`: APE vs time, all runs, mean line per mode | Fig. 3 right |

The integration diagram (Fig. 2) is TikZ inside `report.tex`.
The results table is read from `eval_out/summary_table.tex` at the repository
root (T-011a output, booktabs `tabular`); without it a placeholder table is printed.
