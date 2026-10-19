#!/usr/bin/env bash
# Build the report (2-page body + figure appendix): report/report.pdf (latexmk + pdflatex).
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

# Page rule (T-012h, DIR): the BODY must fit in 2 pages; the figures are in an appendix
# that starts with \clearpage + \onecolumn at \label{app:start} and may take <= 2 pages.
# Checked: app:start is on page <= 3 (body = pages before it, <= 2) and the PDF has
# <= 4 pages (appendix <= 2). The label's page is read from the .aux file
# (hyperref format: \newlabel{app:start}{{<num>}{<page>}...}).
pages=$(pdfinfo report.pdf | awk '/^Pages:/ {print $2}')
app_page=$(sed -n 's/^\\newlabel{app:start}{{[^}]*}{\([0-9]*\)}.*/\1/p' report.aux)
if [[ -z "$app_page" ]]; then
  echo "ERROR: label app:start not found in report.aux" >&2
  exit 1
fi
body=$(( app_page - 1 ))
appendix=$(( pages - body ))
echo "report.pdf: ${pages} page(s) = body ${body} + appendix ${appendix} (appendix starts on page ${app_page})"
if (( body > 2 )); then
  echo "ERROR: the body must be <= 2 pages (appendix must start on page 3)" >&2
  exit 1
fi
if (( appendix > 2 )); then
  echo "ERROR: the appendix must be <= 2 pages (PDF <= 4 pages)" >&2
  exit 1
fi
