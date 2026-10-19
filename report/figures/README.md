# report/figures

Figures used by `report/report.tex` and by its Russian translation `report/report_ru.tex`
(same files). Since T-012h all data figures are in the **appendix** (page 3+, at most 2 pages),
full text width; the 2-page body contains only the TikZ integration diagram (Fig. 1) and the
tables, and refers to the appendix figures (EN "Fig. A1-A3", RU "рис. П1-П3").

| File | Produced by | Used in |
|---|---|---|
| `trajectories.pdf` | `eval/plots.py` on the D-6 route-lever demo, `runs/d6_20261019_055924/plots/trajectories.pdf` (seed 42, noise x4, `shelves_south_wall.yaml`; s* background, estimated heading every 0.5 m). The suptitle is cropped in LaTeX (`trim`); its legend text is in the caption. | Fig. A1 / П1 |
| `paired_t010b.pdf` | `scripts/t011_paired_plot.py` on `runs/t010b_20261019_055924/{x4,x6}` (RND, `knowledge/figures/paired_t010b.pdf`): per-seed paired differences active - baseline with mean and 95 % t-interval | Fig. A2 / П2 |
| `s_star.png` | `python3 -I navigation/src/active_localization/tools/score_maps.py --out <dir>` (best-over-yaw S2* score, 36 yaws). Regenerated in T-012h with the v2 scorer defaults (k_A 20, r_min 0.66, k_r 40, as in Table 2); the T-012a file used the v1 parameters. | Fig. A3 / П3 |

The integration diagram (Fig. 1) is TikZ inside the `.tex` files.
The results table (Table 3, full width) is inlined in the `.tex` files: the x4 summary from
`eval/summarize.py` (`runs/t010b_20261019_055924/x4/summary/summary_table.tex`)
plus the "Path with a marker in view" row and the paired-CI column from RND's T-011
analysis (`08_final_findings.md` §4).
