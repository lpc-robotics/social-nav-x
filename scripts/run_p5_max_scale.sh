#!/usr/bin/env bash
set -Eeuo pipefail

MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="$MPC_WS/evidence/p5/performance"
BINARY="$MPC_WS/install/arena_mpc_core/lib/arena_mpc_core/mpc_benchmark"
CSV="$OUTPUT_DIR/max_scale_1000.csv"
REPORT="$OUTPUT_DIR/max_scale_1000.json"

if [[ ! -x "$BINARY" ]]; then
    echo "MPC benchmark is not built: $BINARY" >&2
    exit 1
fi
mkdir -p "$OUTPUT_DIR"
set +u
source "$MPC_WS/install/setup.bash"
set -u
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$BINARY" 1000 >"$CSV"
python "$MPC_WS/tools/validate_p5_benchmark.py" "$CSV" --output "$REPORT"
