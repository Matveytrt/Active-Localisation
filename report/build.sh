#!/usr/bin/env bash
# Build the 2-page report: report/report.pdf (latexmk + pdflatex).
#   report/build.sh          build
#   report/build.sh clean    remove build artefacts (keeps report.pdf)
# Results (T-012e): the results table is inlined in report.tex (x4 summary from
# eval/summarize.py + RND's paired CIs); figures are in report/figures/.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ "${1:-}" == "clean" ]]; then
  latexmk -c report.tex
  exit 0
fi

# -g: always rebuild
latexmk -g -pdf -interaction=nonstopmode -halt-on-error report.tex

pages=$(pdfinfo report.pdf | awk '/^Pages:/ {print $2}')
echo "report.pdf: ${pages} page(s)"
if (( pages > 2 )); then
  echo "ERROR: the report must be <= 2 pages" >&2
  exit 1
fi
