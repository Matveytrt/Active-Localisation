#!/usr/bin/env bash
# Build the Russian 2-page report: report/report_ru.pdf (latexmk + xelatex).
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
       "the layout will differ and may exceed 2 pages" >&2
fi

# -g: always rebuild
latexmk -g -xelatex -interaction=nonstopmode -halt-on-error report_ru.tex

if grep -q "Missing character" report_ru.log; then
  echo "ERROR: missing glyphs (see 'Missing character' in report_ru.log)" >&2
  exit 1
fi

pages=$(pdfinfo report_ru.pdf | awk '/^Pages:/ {print $2}')
echo "report_ru.pdf: ${pages} page(s)"
if (( pages > 2 )); then
  echo "ERROR: the report must be <= 2 pages" >&2
  exit 1
fi
