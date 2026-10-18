#!/usr/bin/env bash
# Build the 2-page report: report/report.pdf (latexmk + pdflatex).
#   report/build.sh          build
#   report/build.sh clean    remove build artefacts (keeps report.pdf)
# The results table is read from ../eval_out/summary_table.tex (repo root)
# when it exists (T-011a); otherwise a placeholder table is printed.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ "${1:-}" == "clean" ]]; then
  latexmk -c report.tex
  exit 0
fi

# -g: always rebuild (latexmk does not notice when ../eval_out/summary_table.tex appears)
latexmk -g -pdf -interaction=nonstopmode -halt-on-error report.tex

pages=$(pdfinfo report.pdf | awk '/^Pages:/ {print $2}')
echo "report.pdf: ${pages} page(s)"
if (( pages > 2 )); then
  echo "ERROR: the report must be <= 2 pages" >&2
  exit 1
fi
