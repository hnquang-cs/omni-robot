#!/usr/bin/env bash
# Fix Stage 9 N/A values by auditing and re-running failed trials.
#
# Usage:
#   fix_stage9_na_and_rerun.sh --core-only          # corridor_static only (gmapping + hector, rep 1-3)
#   fix_stage9_na_and_rerun.sh --full               # all scenarios
#   fix_stage9_na_and_rerun.sh --single <scenario> <algorithm> <rep>

set -euo pipefail

MODE="core-only"
SINGLE_SCENARIO=""
SINGLE_ALGORITHM=""
SINGLE_REP=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --core-only) MODE="core-only"; shift ;;
    --full)      MODE="full";      shift ;;
    --single)
      if [[ $# -lt 4 || -z "${2:-}" || -z "${3:-}" || -z "${4:-}" ]]; then
        echo "FAIL: --single requires <scenario> <algorithm> <rep>"
        exit 2
      fi
      MODE="single"
      SINGLE_SCENARIO="$2"
      SINGLE_ALGORITHM="$3"
      SINGLE_REP="$4"
      shift 4
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 --core-only | --full | --single <scenario> <algorithm> <rep>"
      exit 2
      ;;
  esac
done

PKG_PATH="$(rospack find slam_benchmark)"
SCRIPTS="${PKG_PATH}/scripts"
BASE="${PKG_PATH}/results/stage9"
CSV_DIR="${BASE}/csv"
LOG_DIR="${BASE}/logs"
mkdir -p "${LOG_DIR}"

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RERUN_LOG="${LOG_DIR}/fix_na_rerun_${RUN_ID}.log"

log_line() {
  echo "[$(date -Is)] $*" | tee -a "${RERUN_LOG}"
}

# ── Step 1: Audit current state ───────────────────────────────────────────────
log_line "=== Step 1: Audit ==="
python3 "${SCRIPTS}/audit_stage9_na.py" | tee -a "${RERUN_LOG}"

RERUN_CSV="${CSV_DIR}/trials_to_rerun.csv"
if [[ ! -f "${RERUN_CSV}" ]]; then
  log_line "FAIL: trials_to_rerun.csv not found after audit"
  exit 1
fi

# ── Step 2: Select trials to run ─────────────────────────────────────────────
CORE_SCENARIOS=(corridor_static)
CORE_ALGORITHMS=(gmapping_lidar hector_lidar)
CORE_REPS=(1 2 3)

ALL_SCENARIOS=(corridor_static open_room_obstacles narrow_turn)
ALL_ALGORITHMS=(gmapping_lidar hector_lidar)
ALL_REPS=(1 2 3)

run_single_trial() {
  local scenario="$1" algorithm="$2" rep="$3"
  log_line "--- Rerunning: ${scenario}/${algorithm}/rep_${rep} ---"
  if "${SCRIPTS}/run_stage9_lidar_benchmark.sh" --single "${scenario}" "${algorithm}" "${rep}"; then
    log_line "PASS: ${scenario}/${algorithm}/rep_${rep}"
    return 0
  else
    local rc=$?
    log_line "WARN: ${scenario}/${algorithm}/rep_${rep} returned rc=${rc}"
    return 0  # non-fatal: continue with next trial
  fi
}

# ── Step 3: Run trials based on mode ─────────────────────────────────────────
log_line "=== Step 3: Rerun mode=${MODE} ==="

case "${MODE}" in
  core-only)
    log_line "Running core trials: corridor_static x {gmapping_lidar,hector_lidar} x rep{1,2,3}"
    for scenario in "${CORE_SCENARIOS[@]}"; do
      for algorithm in "${CORE_ALGORITHMS[@]}"; do
        for rep in "${CORE_REPS[@]}"; do
          run_single_trial "${scenario}" "${algorithm}" "${rep}"
        done
      done
    done
    ;;

  full)
    log_line "Running all trials: all scenarios x all algorithms x rep{1,2,3}"
    for scenario in "${ALL_SCENARIOS[@]}"; do
      for algorithm in "${ALL_ALGORITHMS[@]}"; do
        for rep in "${ALL_REPS[@]}"; do
          run_single_trial "${scenario}" "${algorithm}" "${rep}"
        done
      done
    done
    ;;

  single)
    run_single_trial "${SINGLE_SCENARIO}" "${SINGLE_ALGORITHM}" "${SINGLE_REP}"
    ;;
esac

# ── Step 4: Aggregate results ─────────────────────────────────────────────────
log_line "=== Step 4: Aggregate ==="
python3 "${SCRIPTS}/aggregate_stage9_results.py" | tee -a "${RERUN_LOG}"

# ── Step 5: Run audit again to show what remains ─────────────────────────────
log_line "=== Step 5: Post-rerun audit ==="
python3 "${SCRIPTS}/audit_stage9_na.py" | tee -a "${RERUN_LOG}"

# ── Summary ───────────────────────────────────────────────────────────────────
log_line "=== Done ==="
log_line "Raw CSV:  ${CSV_DIR}/slam_comparison_raw.csv"
log_line "Mean CSV: ${CSV_DIR}/slam_comparison_mean.csv"
log_line "Log:      ${RERUN_LOG}"

echo ""
echo "Results:"
echo "  Raw:  ${CSV_DIR}/slam_comparison_raw.csv"
echo "  Mean: ${CSV_DIR}/slam_comparison_mean.csv"
echo ""
echo "Quick view:"
MEAN_CSV="${CSV_DIR}/slam_comparison_mean.csv"
python3 - "${MEAN_CSV}" <<'PY'
import csv, sys
from pathlib import Path
mean_csv = Path(sys.argv[1])
if not mean_csv.is_file():
    print(f"(mean CSV not found: {mean_csv})")
    sys.exit(0)
rows = list(csv.DictReader(mean_csv.open()))
print(f"{'Scenario':<22} {'Algorithm':<16} {'Success':>8} {'ATE Valid':>10} {'ATE RMSE':>10} {'Map Valid':>10} {'Runtime':>10}")
print("-" * 92)
for r in rows:
    print(f"{r['scenario']:<22} {r['algorithm']:<16} {r['n_success']:>3}/{r['n_trials']:<4}  {r['n_metric_valid_ate']:>9} {r['ate_rmse_mean']:>10} {r['n_metric_valid_map']:>9} {r['runtime_sec_mean']:>10}")
PY
