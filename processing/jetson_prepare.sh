#!/usr/bin/env bash
#
# Jetson side of the Husky evaluation (plan/husky_stage1_runbook.md, step 2): extract the IR stereo
# from the raw bag and install the reference trajectory that was built on the PC.
#
#   processing/jetson_prepare.sh [BAG_DIR] [SEQ_NAME]
#       BAG_DIR   default ~/datasets/external_raw/husky_20261023   (metadata.yaml + .db3)
#       SEQ_NAME  default husky_20261023  -> ~/datasets/ext/SEQ_NAME (what run_matrix.py ext replays)
#
# The reference is NOT recomputed here: it comes from the repository
# (jetson-orbslam3-results-husky/reference/sequence/gt.csv). This script checks that the images it
# extracts are exactly the ones the reference was built for (identical times.txt and settings.yaml).
# Needs only the light venv (processing/setup_venv.sh --light). The bag is only read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-$HOME/datasets/external_raw/husky_20261023}"
NAME="${2:-husky_20261023}"
SEQ="$HOME/datasets/ext/$NAME"
REF="$ROOT/jetson-orbslam3-results-husky/reference/sequence"
PY="${PY:-$HOME/venvs/ext/bin/python}"

[ -f "$BAG/metadata.yaml" ] || { echo "ERROR: no metadata.yaml in $BAG"; exit 1; }
[ -f "$REF/gt.csv" ] || { echo "ERROR: no reference at $REF (git pull?)"; exit 1; }
"$PY" -c "import rosbags, cv2, numpy" || { echo "ERROR: run processing/setup_venv.sh --light first"; exit 1; }

echo "==> 1/3 IR stereo -> $SEQ"
if [ -f "$SEQ/settings.yaml" ] && [ -f "$SEQ/times.txt" ]; then
  echo "    already extracted ($(wc -l < "$SEQ/times.txt") pairs)"
else
  "$PY" -I "$ROOT/processing/bag_to_euroc.py" "$BAG" "$SEQ"
fi

echo "==> 2/3 check: the extracted sequence is the one the reference was built for"
ok=1
cmp -s "$SEQ/times.txt" "$REF/times.txt" && echo "    times.txt     identical ($(wc -l < "$SEQ/times.txt") stamps)" \
  || { echo "    ERROR: times.txt differs from the reference's"; ok=0; }
cmp -s "$SEQ/settings.yaml" "$REF/settings.yaml" && echo "    settings.yaml identical" \
  || { echo "    ERROR: settings.yaml differs (calibration or ORB settings changed)"; ok=0; }
n0=$(ls "$SEQ/mav0/cam0/data" | wc -l); n1=$(ls "$SEQ/mav0/cam1/data" | wc -l)
[ "$n0" = "$n1" ] && [ "$n0" = "$(wc -l < "$SEQ/times.txt")" ] && echo "    images        $n0 left + $n1 right" \
  || { echo "    ERROR: image count mismatch (left $n0, right $n1)"; ok=0; }
[ $ok = 1 ] || exit 1

echo "==> 3/3 install the reference (gt.csv + provenance)"
cp "$REF/gt.csv" "$REF/reference_info.json" "$SEQ/"
echo "    $SEQ/gt.csv: $(($(wc -l < "$SEQ/gt.csv") - 1)) poses"
echo
echo "Ready. Next (runbook step 4):  python3 repro/run_matrix.py ext --dry-run"
