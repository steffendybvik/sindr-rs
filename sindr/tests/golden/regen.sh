#!/usr/bin/env bash
# Regenerates every <case>.golden from <case>.cir + <case>.ctl with ngspice.
#
#   sindr/tests/golden/regen.sh            # all cases
#   sindr/tests/golden/regen.sh op_diode   # one case
#
# Each .cir is the netlist both simulators read; the .ctl holds the ngspice
# analysis and a `wrdata @OUT@ ...` line naming the vectors to record.
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v ngspice >/dev/null; then
  echo "ngspice not found on PATH (macOS: brew install ngspice)" >&2
  exit 1
fi

cases=("$@")
if [ ${#cases[@]} -eq 0 ]; then
  for ctl in *.ctl; do cases+=("${ctl%.ctl}"); done
fi

for case in "${cases[@]}"; do
  deck=".regen-$case.cir"
  {
    grep -viE '^[[:space:]]*\.end[[:space:]]*$' "$case.cir"
    echo ".control"
    echo "set wr_singlescale"
    echo "set wr_vecnames"
    sed "s|@OUT@|$case.golden|" "$case.ctl"
    echo ".endc"
    echo ".end"
  } >"$deck"
  if ! ngspice -b "$deck" >".regen-$case.log" 2>&1 || [ ! -s "$case.golden" ]; then
    echo "ngspice failed on $case; see .regen-$case.log" >&2
    rm -f "$deck"
    exit 1
  fi
  rm -f "$deck" ".regen-$case.log"
  echo "wrote $case.golden"
done
