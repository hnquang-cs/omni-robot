#!/usr/bin/env bash

set -euo pipefail

YES=false
DRY_RUN=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --yes|-y) YES=true ;;
    --dry-run) DRY_RUN=true ;;
    *) echo "Unknown option: $1"; exit 2 ;;
  esac
  shift
done

PKG_PATH="$(rospack find slam_benchmark)"
SCENARIOS=(corridor_static open_room_obstacles narrow_turn)
ALGORITHMS=(gmapping_conservative gmapping_fast hector)
REPETITIONS=(1 2 3)
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_LOG="${PKG_PATH}/results/stage8/logs/run_${RUN_ID}.log"
mkdir -p "${PKG_PATH}/results/stage8/logs"

echo "Stage 8 benchmark run ${RUN_ID}" | tee -a "${RUN_LOG}"
echo "Scenarios: ${SCENARIOS[*]}" | tee -a "${RUN_LOG}"
echo "Algorithms: ${ALGORITHMS[*]}" | tee -a "${RUN_LOG}"
echo "Repetitions: ${REPETITIONS[*]}" | tee -a "${RUN_LOG}"
echo "YAML files are documented in config/. This runner uses hard-coded lists to avoid fragile YAML parsing in bash." | tee -a "${RUN_LOG}"

if [[ "${YES}" != true ]]; then
  read -r -p "Run all Stage 8 trials now? This can take a long time. Type YES: " answer
  if [[ "${answer}" != "YES" ]]; then
    echo "Aborted."
    exit 0
  fi
fi

for scenario in "${SCENARIOS[@]}"; do
  for algorithm in "${ALGORITHMS[@]}"; do
    for rep in "${REPETITIONS[@]}"; do
      cmd=(rosrun slam_benchmark run_single_slam_trial.sh "${scenario}" "${algorithm}" "${rep}")
      echo "command: ${cmd[*]}" | tee -a "${RUN_LOG}"
      if [[ "${DRY_RUN}" == true ]]; then
        continue
      fi
      if ! "${cmd[@]}" | tee -a "${RUN_LOG}"; then
        echo "WARN: trial failed: ${scenario}/${algorithm}/rep_${rep}" | tee -a "${RUN_LOG}"
      fi
      sleep 6
    done
  done
done

echo "Post-processing Stage 8 outputs" | tee -a "${RUN_LOG}"
for cmd in \
  "rosrun slam_benchmark aggregate_stage8_results.py" \
  "rosrun slam_benchmark plot_stage8_results.py" \
  "rosrun slam_benchmark generate_latex_tables.py"; do
  echo "command: ${cmd}" | tee -a "${RUN_LOG}"
  if [[ "${DRY_RUN}" != true ]]; then
    bash -lc "${cmd}" | tee -a "${RUN_LOG}" || true
  fi
done

echo "Summary CSV: ${PKG_PATH}/results/stage8/csv/stage8_summary.csv" | tee -a "${RUN_LOG}"
echo "Run log: ${RUN_LOG}" | tee -a "${RUN_LOG}"
