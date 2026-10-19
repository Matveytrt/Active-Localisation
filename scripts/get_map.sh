#!/usr/bin/env bash
# Download the RTAB-Map database used in the experiments from a GitHub Release asset and verify
# its SHA-256 (D-11). The DB is not in git (~150 MB, navigation/maps/*.db is ignored).
#
#   scripts/get_map.sh [--url URL] [--sha256 HEX] [--out DIR] [--force] [--no-pristine]
#   scripts/get_map.sh --verify-only [--sha256 HEX] [--out DIR]
#   scripts/get_map.sh --print-sha256 FILE        # maintainers: hash to publish with the release
#
# Result: <out>/astribot_sim.db (default out: navigation/maps) and, unless --no-pristine,
# <out>/astribot_sim.pristine.db, the copy scripts/run_experiment.sh restores before every run.
#
# URL and checksum, first one found wins:
#   --url / --sha256  >  $AL_MAP_URL / $AL_MAP_SHA256  >  the defaults below / the manifest
#   navigation/maps/astribot_sim.db.sha256 ("<hex>  astribot_sim.db", as `sha256sum` prints it).
# A URL ending in .gz is gunzipped after the download; the checksum is always the one of the
# final .db file. A placeholder URL or checksum (TODO) stops the script before anything is
# downloaded. file:// URLs work too (offline copies, tests).
#
# Exit codes: 0 ok, 2 bad arguments / placeholder / missing tool, 3 checksum mismatch,
# 4 an existing different DB would be overwritten (use --force), 5 download failed.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MAP_NAME=astribot_sim.db
# TODO(ENG/LEAD): upload the DB as a release asset, then put its URL here (or in AL_MAP_URL).
DEFAULT_URL='https://github.com/Matveytrt/Active-Localisation/releases/download/TODO-TAG/astribot_sim.db'
OUT="$REPO_ROOT/navigation/maps"
URL=${AL_MAP_URL:-$DEFAULT_URL}
SHA=${AL_MAP_SHA256:-}
FORCE=0
PRISTINE=1
VERIFY_ONLY=0

die() { echo "get_map.sh: $1" >&2; exit "${2:-2}"; }

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1
  elif command -v shasum >/dev/null 2>&1; then shasum -a 256 "$1" | cut -d' ' -f1
  else die "need sha256sum or shasum"; fi
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --url) URL=${2:?--url needs a value}; shift 2 ;;
    --sha256) SHA=${2:?--sha256 needs a value}; shift 2 ;;
    --out) OUT=${2:?--out needs a value}; shift 2 ;;
    --force) FORCE=1; shift ;;
    --no-pristine) PRISTINE=0; shift ;;
    --verify-only) VERIFY_ONLY=1; shift ;;
    --print-sha256) [[ -f ${2:-} ]] || die "--print-sha256 needs an existing file"; sha256_of "$2"; exit 0 ;;
    -h|--help) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "unknown argument '$1' (see --help)" ;;
  esac
done

MANIFEST="$REPO_ROOT/navigation/maps/$MAP_NAME.sha256"
if [[ -z $SHA && -f $MANIFEST ]]; then SHA=$(awk 'NF {print $1; exit}' "$MANIFEST"); fi
SHA=$(printf '%s' "$SHA" | tr 'A-F' 'a-f')
[[ $SHA =~ ^[0-9a-f]{64}$ ]] \
  || die "no valid SHA-256 (got '${SHA:-none}'): pass --sha256 / AL_MAP_SHA256 or fill $MANIFEST (TODO(ENG): publish it with the release)"
[[ $SHA =~ ^0+$ ]] && die "the checksum in $MANIFEST is still the all-zero placeholder (TODO(ENG))"

DB="$OUT/$MAP_NAME"
mkdir -p "$OUT"

if [[ -f $DB ]]; then
  HAVE=$(sha256_of "$DB")
  if [[ $HAVE == "$SHA" ]]; then
    echo "OK: $DB already matches sha256 $SHA"
    DONE=1
  elif [[ -f $OUT/astribot_sim.pristine.db && $FORCE -eq 0 ]] \
    && [[ $(sha256_of "$OUT/astribot_sim.pristine.db") == "$SHA" ]]; then
    # Localization runs write into the working DB; the pristine copy is what counts.
    echo "OK: $OUT/astribot_sim.pristine.db matches sha256 $SHA ($MAP_NAME was modified by a run)"
    exit 0
  elif [[ $VERIFY_ONLY -eq 1 ]]; then
    die "$DB has sha256 $HAVE, expected $SHA" 3
  elif [[ $FORCE -eq 0 ]]; then
    die "$DB exists and differs (sha256 $HAVE; modified by a run, or your own map?). Use --force to replace it." 4
  fi
elif [[ $VERIFY_ONLY -eq 1 ]]; then
  die "$DB does not exist" 3
fi

if [[ $VERIFY_ONLY -eq 1 ]]; then exit 0; fi

if [[ ${DONE:-0} -ne 1 ]]; then
  [[ $URL == *TODO* ]] \
    && die "the map URL is still a placeholder ($URL): pass --url / AL_MAP_URL (TODO(ENG): release asset)"
  if command -v curl >/dev/null 2>&1; then
    if [[ -t 2 ]]; then PROGRESS=--progress-bar; else PROGRESS=-sS; fi
    FETCH=(curl -fL "$PROGRESS" --retry 3 --retry-delay 2 -o)
  elif command -v wget >/dev/null 2>&1; then
    FETCH=(wget -q -O)
  else
    die "need curl or wget"
  fi
  TMP=$(mktemp "$OUT/.$MAP_NAME.XXXXXX.part")
  trap 'rm -f "$TMP" "$TMP.db"' EXIT
  echo "downloading $URL"
  "${FETCH[@]}" "$TMP" "$URL" || die "download failed: $URL" 5
  if [[ $URL == *.gz ]]; then
    command -v gzip >/dev/null 2>&1 || die "need gzip for a .gz asset"
    gzip -dc "$TMP" > "$TMP.db" || die "cannot gunzip the download" 5
    mv -f "$TMP.db" "$TMP"
  fi
  GOT=$(sha256_of "$TMP")
  if [[ $GOT != "$SHA" ]]; then
    die "checksum mismatch: downloaded $GOT, expected $SHA (file discarded)" 3
  fi
  mv -f "$TMP" "$DB"
  trap - EXIT
  echo "OK: $DB (sha256 $SHA)"
fi

if [[ $PRISTINE -eq 1 ]]; then
  cp -f "$DB" "$OUT/astribot_sim.pristine.db"
  echo "pristine copy: $OUT/astribot_sim.pristine.db"
fi
