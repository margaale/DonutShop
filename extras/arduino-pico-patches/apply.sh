#!/usr/bin/env bash
# Applies the arduino-pico fixes in this folder to every installed copy of the matching core
# version (arduino-cli profiles install cores under ~/.arduino15/internal/, the Boards Manager
# under ~/.arduino15/packages/). Safe to run more than once.
#
#   extras/arduino-pico-patches/apply.sh [arduino-data-dir]
#
# Why: arduino-pico's FreeRTOS networking races the lwIP task against the application tasks
# (see README.md in this folder). Drop a version's folder once upstream ships the fix.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
data="${1:-$HOME/.arduino15}"
status=0

for vdir in "$here"/*/; do
  version="$(basename "$vdir")"
  shopt -s nullglob
  cores=("$data"/internal/rp2040_rp2040_"$version"_* "$data"/packages/rp2040/hardware/rp2040/"$version")
  shopt -u nullglob
  for core in "${cores[@]}"; do
    [[ -d "$core/cores/rp2040" ]] || continue
    for p in "$vdir"*.patch; do
      name="$(basename "$p")"
      if patch -d "$core" -p1 -R --dry-run --silent -f < "$p" >/dev/null 2>&1; then
        echo "already applied: $name -> $core"
      elif patch -d "$core" -p1 --dry-run --silent -f < "$p" >/dev/null 2>&1; then
        patch -d "$core" -p1 --silent -f < "$p"
        echo "applied: $name -> $core"
      else
        echo "FAILED: $name does not apply to $core" >&2
        status=1
      fi
    done
  done
done
exit $status
