#!/usr/bin/env bash
# APE + RPE (evo) for one recorded run.
#
#   eval/run_evo.sh <run_dir> [--align none|se3|origin|posyaw] [options]
#
# Inputs : <run_dir>/gt.tum, <run_dir>/est.tum   (TUM, seconds)
# Outputs: <run_dir>/evo_<align>/
#            ape.zip rpe.zip            evo results (stats.json inside)
#            ape_raw.png ape_map.png    plots (or ape.pdf with --plot_format pdf)
#            rpe_raw.png rpe_map.png
#            ape.log rpe.log            evo console output
#            commands.txt               exact evo command lines used
#
# Options:
#   --align A        none (default) | se3 (evo -a, Umeyama SE(3)) |
#                    origin (evo --align_origin) |
#                    posyaw (yaw + xy translation, eval/align_posyaw.py, then
#                    evo without alignment)
#   --t_max_diff S   max |t_gt - t_est| for association, default 0.02 s
#                    (GT 50 Hz -> nearest GT sample is <= 0.01 s away)
#   --sync M         interpolation (default: GT interpolated at est stamps)
#                    | nearest_time
#   --delta D        RPE delta in metres, default 1
#   --plot_format F  png (default) | pdf | none
#   --3d             do NOT project to the xy plane (default: project, since
#                    the robot is planar and GT z (spawn height) != est z)
#   --user_settings  use ~/.evo/settings.json as is. By default evo runs with
#                    a throw-away HOME so that its plot backend is Agg
#                    (evo ignores MPLBACKEND and takes the backend from
#                    ~/.evo/settings.json). Metrics are identical either way.
#   --gt F / --est F override input file names (relative to run_dir)
set -euo pipefail

usage() { sed -n '2,32p' "$0"; exit "${1:-0}"; }

[[ $# -ge 1 ]] || usage 1
case "$1" in -h|--help) usage 0 ;; esac
RUN_DIR=$(cd "$1" && pwd); shift

ALIGN=none; T_MAX_DIFF=0.02; SYNC=interpolation; DELTA=1; PLOT_FORMAT=png
PROJECT=1; USER_SETTINGS=0; GT_NAME=gt.tum; EST_NAME=est.tum
while [[ $# -gt 0 ]]; do
  case "$1" in
    --align) ALIGN=$2; shift 2 ;;
    --t_max_diff) T_MAX_DIFF=$2; shift 2 ;;
    --sync) SYNC=$2; shift 2 ;;
    --delta) DELTA=$2; shift 2 ;;
    --plot_format) PLOT_FORMAT=$2; shift 2 ;;
    --3d) PROJECT=0; shift ;;
    --user_settings) USER_SETTINGS=1; shift ;;
    --gt) GT_NAME=$2; shift 2 ;;
    --est) EST_NAME=$2; shift 2 ;;
    -h|--help) usage 0 ;;
    *) echo "unknown option: $1" >&2; usage 1 ;;
  esac
done

HERE=$(cd "$(dirname "$0")" && pwd)
GT="$RUN_DIR/$GT_NAME"
EST="$RUN_DIR/$EST_NAME"
for f in "$GT" "$EST"; do
  [[ -s "$f" ]] || { echo "missing or empty: $f" >&2; exit 2; }
done
command -v evo_ape >/dev/null || {
  echo "evo not found: pip install evo  (see eval/README.md)" >&2; exit 3; }

OUT="$RUN_DIR/evo_$ALIGN"
mkdir -p "$OUT"
rm -f "$OUT"/ape.zip "$OUT"/rpe.zip "$OUT"/*.png "$OUT"/*.pdf

ALIGN_ARGS=()
case "$ALIGN" in
  none) ;;
  se3) ALIGN_ARGS=(-a) ;;
  origin) ALIGN_ARGS=(--align_origin) ;;
  posyaw)
    python3 "$HERE/align_posyaw.py" "$GT" "$EST" "$OUT/est_posyaw.tum" \
      --t_max_diff "$T_MAX_DIFF" --sync_method "$SYNC" | tee "$OUT/posyaw.log"
    EST="$OUT/est_posyaw.tum" ;;
  *) echo "bad --align: $ALIGN (none|se3|origin|posyaw)" >&2; exit 1 ;;
esac

COMMON=(--sync_method "$SYNC" --t_max_diff "$T_MAX_DIFF" --no_warnings)
[[ $PROJECT -eq 1 ]] && COMMON+=(--project_to_plane xy)

APE_PLOT=(); RPE_PLOT=()
case "$PLOT_FORMAT" in
  none) ;;
  png|pdf)
    APE_PLOT=(--plot_mode xy --save_plot "$OUT/ape.$PLOT_FORMAT")
    RPE_PLOT=(--plot_mode xy --save_plot "$OUT/rpe.$PLOT_FORMAT") ;;
  *) echo "bad --plot_format: $PLOT_FORMAT (png|pdf|none)" >&2; exit 1 ;;
esac

# --- isolate evo settings so that the plot backend is Agg (headless) -------
ENV_PREFIX=(env MPLBACKEND=Agg)
if [[ $USER_SETTINGS -eq 0 ]]; then
  REAL_HOME=$HOME
  EVO_HOME=$(mktemp -d)
  trap 'rm -rf "$EVO_HOME"' EXIT
  # keep `pip install --user` packages and the matplotlib cache reachable
  ENV_PREFIX+=(HOME="$EVO_HOME" DISPLAY=
               PYTHONUSERBASE="${PYTHONUSERBASE:-$REAL_HOME/.local}"
               MPLCONFIGDIR="${MPLCONFIGDIR:-$REAL_HOME/.config/matplotlib}")
fi

APE_CMD=(evo_ape tum "$GT" "$EST" -r trans_part "${ALIGN_ARGS[@]}"
         "${COMMON[@]}" --save_results "$OUT/ape.zip" "${APE_PLOT[@]}")
RPE_CMD=(evo_rpe tum "$GT" "$EST" -r trans_part --delta "$DELTA"
         --delta_unit m --pairs_from_reference "${ALIGN_ARGS[@]}"
         "${COMMON[@]}" --save_results "$OUT/rpe.zip" "${RPE_PLOT[@]}")

{
  echo "# evo $(evo pkg --version 2>/dev/null || echo unknown)"
  printf '%q ' "${APE_CMD[@]}"; echo
  printf '%q ' "${RPE_CMD[@]}"; echo
} > "$OUT/commands.txt"

echo "=== APE ($ALIGN) ==="
"${ENV_PREFIX[@]}" "${APE_CMD[@]}" 2>&1 | tee "$OUT/ape.log"
echo "=== RPE ($ALIGN, delta ${DELTA} m) ==="
"${ENV_PREFIX[@]}" "${RPE_CMD[@]}" 2>&1 | tee "$OUT/rpe.log"

[[ -s "$OUT/ape.zip" && -s "$OUT/rpe.zip" ]] || {
  echo "evo did not produce results in $OUT" >&2; exit 4; }

python3 - "$OUT" "$ALIGN" <<'EOF'
import json, sys, zipfile
out, align = sys.argv[1], sys.argv[2]
def st(name):
    with zipfile.ZipFile('%s/%s.zip' % (out, name)) as z:
        return json.loads(z.read('stats.json'))
a, r = st('ape'), st('rpe')
print('SUMMARY align=%s  APE rmse=%.4f mean=%.4f max=%.4f  |  '
      'RPE rmse=%.4f mean=%.4f  [m]' % (align, a['rmse'], a['mean'],
                                        a['max'], r['rmse'], r['mean']))
EOF
echo "results: $OUT"
