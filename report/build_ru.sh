#!/usr/bin/env bash
# Build the Russian report (2-page body + figure appendix): report/report_ru.pdf (latexmk + xelatex).
#   report/build_ru.sh          build
#   report/build_ru.sh clean    remove build artefacts (keeps report_ru.pdf)
# report_ru.tex is a translation of report.tex (master); same figures (report/figures/).
# Needs: xelatex, fontspec, polyglossia, fonts "Liberation Serif", "DejaVu Sans",
# "DejaVu Sans Mono" (fc-list), Russian hyphenation patterns (texlive-lang-cyrillic).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ "${1:-}" == "clean" ]]; then
  latexmk -xelatex -c report_ru.tex
  exit 0
fi

if ! kpsewhich hyph-ru.tex >/dev/null; then
  echo "WARNING: Russian hyphenation patterns not found (install texlive-lang-cyrillic);" \
       "the layout will differ and the body may exceed 2 pages" >&2
fi

# -g: always rebuild
latexmk -g -xelatex -interaction=nonstopmode -halt-on-error report_ru.tex

if grep -q "Missing character" report_ru.log; then
  echo "ERROR: missing glyphs (see 'Missing character' in report_ru.log)" >&2
  exit 1
fi

# Page rule (T-012h, DIR): the BODY must fit in 2 pages; the figures are in an appendix
# that starts with \clearpage + \onecolumn at \label{app:start} and may take <= 2 pages.
# Checked: app:start is on page <= 3 (body = pages before it, <= 2) and the PDF has
# <= 4 pages (appendix <= 2). The label's page is read from the .aux file
# (hyperref format: \newlabel{app:start}{{<num>}{<page>}...}).
pages=$(pdfinfo report_ru.pdf | awk '/^Pages:/ {print $2}')
app_page=$(sed -n 's/^\\newlabel{app:start}{{[^}]*}{\([0-9]*\)}.*/\1/p' report_ru.aux)
if [[ -z "$app_page" ]]; then
  echo "ERROR: label app:start not found in report_ru.aux" >&2
  exit 1
fi
body=$(( app_page - 1 ))
appendix=$(( pages - body ))
echo "report_ru.pdf: ${pages} page(s) = body ${body} + appendix ${appendix} (appendix starts on page ${app_page})"
if (( body > 2 )); then
  echo "ERROR: the body must be <= 2 pages (appendix must start on page 3)" >&2
  exit 1
fi
if (( appendix > 2 )); then
  echo "ERROR: the appendix must be <= 2 pages (PDF <= 4 pages)" >&2
  exit 1
fi
