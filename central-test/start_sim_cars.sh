#!/usr/bin/env bash
# Start N simulated cars (default 3) on ports 8081.., each with its own copy of the policy
# in sim-cars/carN/student_policy.py, so policy uploads from the hub never touch the repo.
#   ./start_sim_cars.sh            # 3 cars
#   ./start_sim_cars.sh 4          # 4 cars
#   CAR_REPO=/path/to/AOE-DR ./start_sim_cars.sh
# Ctrl+C stops them all.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CAR_REPO="${CAR_REPO:-$HERE/../AOE-DR}"
N="${1:-3}"
PY="${PYTHON:-python3}"
SEED_POLICY="$CAR_REPO/policy-10-22.py"
[ -f "$SEED_POLICY" ] || SEED_POLICY="$CAR_REPO/student_policy.py"

pids=()
trap 'kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null' EXIT INT TERM
for i in $(seq 1 "$N"); do
  dir="$HERE/sim-cars/car$i"
  mkdir -p "$dir"
  [ -f "$dir/student_policy.py" ] || cp "$SEED_POLICY" "$dir/student_policy.py"
  port=$((8080 + i))
  ( cd "$dir" && PYTHONPATH="$CAR_REPO" exec "$PY" -m aoe ui --sim \
        --policy "$dir/student_policy.py" --host 127.0.0.1 --port "$port" ) \
      > "$dir/ui.log" 2>&1 &
  pids+=($!)
  echo "sim car $i: http://127.0.0.1:$port  (log: sim-cars/car$i/ui.log)"
done
echo "now run: python3 hub.py   and open http://localhost:9000   (Ctrl+C here stops the cars)"
wait
