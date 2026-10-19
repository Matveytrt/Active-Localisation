# report/figures

Figures used by `report/report.tex` and by its Russian translation `report/report_ru.tex`
(same files; in `report_ru.tex` Fig. 1 is set at height 3.2 cm instead of 3.5 cm to keep the
longer Russian text within 2 pages).

| File | Produced by | Used in |
|---|---|---|
| `trajectories.pdf` | `eval/plots.py` on the D-6 route-lever demo, `runs/d6_20261019_055924/plots/trajectories.pdf` (seed 42, noise x4, `shelves_south_wall.yaml`; s* background, estimated heading every 0.5 m). The suptitle is cropped in LaTeX (`trim`). | Fig. 1a (results) |
| `paired_t010b.pdf` | `scripts/t011_paired_plot.py` on `runs/t010b_20261019_055924/{x4,x6}` (RND, `knowledge/figures/paired_t010b.pdf`): per-seed paired differences active - baseline with mean and 95 % t-interval | Fig. 1b (results) |
| `s_star.png` | `navigation/src/active_localization/tools/score_maps.py --out <dir>` (T-009a; best-over-yaw S2* score, 36 yaws) | not used since T-012e (removed to fit 2 pages; the same field is the background of Fig. 1a) |

The integration diagram (Fig. 2) is TikZ inside `report.tex`.
The results table (Table 3) is inlined in `report.tex`: the x4 summary from
`eval/summarize.py` (`runs/t010b_20261019_055924/x4/summary/summary_table.tex`)
plus the "Path with marker" row and the paired-CI column from RND's T-011
analysis (`08_final_findings.md` §4).
