#!/usr/bin/env bash
# Compute ATE/RPE: always runs Python fallback; uses evo when available.

set -euo pipefail

GT="${1:-}"
EST="${2:-}"
OUT_DIR="${3:-}"

if [[ -z "${GT}" || -z "${EST}" || -z "${OUT_DIR}" ]]; then
  echo "Usage: compute_ate_rpe.sh <groundtruth.tum> <estimated.tum> <output_dir>"
  exit 2
fi

mkdir -p "${OUT_DIR}"
METRICS_CSV="${OUT_DIR}/trajectory_metrics.csv"
STATUS_TXT="${OUT_DIR}/metrics_status.txt"
APE_TXT="${OUT_DIR}/evo_ape_output.txt"
RPE_TXT="${OUT_DIR}/evo_rpe_output.txt"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_FALLBACK="${SCRIPT_DIR}/compute_trajectory_metrics.py"

write_na_metrics() {
  local reason="$1"
  {
    echo "metric,value"
    echo "ate_rmse,N/A"
    echo "ate_mean,N/A"
    echo "ate_max,N/A"
    echo "ate_std,N/A"
    echo "rpe_rmse,N/A"
    echo "rpe_mean,N/A"
    echo "rpe_max,N/A"
    echo "rpe_std,N/A"
    echo "trajectory_metrics_source,none"
    echo "na_reason,${reason}"
  } > "${METRICS_CSV}"
  echo "FAIL: ${reason}" > "${STATUS_TXT}"
}

if [[ ! -s "${GT}" ]]; then
  write_na_metrics "groundtruth_missing_or_empty:${GT}"
  echo "WARN: ${GT} is missing or empty"
  exit 0
fi

if [[ ! -s "${EST}" ]]; then
  write_na_metrics "estimated_missing_or_empty:${EST}"
  echo "WARN: ${EST} is missing or empty"
  exit 0
fi

# Step 1: Always run the Python fallback to produce a baseline trajectory_metrics.csv.
if [[ -x "${PYTHON_FALLBACK}" ]]; then
  if python3 "${PYTHON_FALLBACK}" \
      --groundtruth "${GT}" \
      --estimated "${EST}" \
      --output "${METRICS_CSV}"; then
    echo "INFO: python_fallback trajectory metrics written"
  else
    echo "WARN: python_fallback returned non-zero; metrics may be N/A"
  fi
else
  write_na_metrics "python_fallback_script_not_found:${PYTHON_FALLBACK}"
  echo "WARN: ${PYTHON_FALLBACK} not found; install suggestion: see scripts/compute_trajectory_metrics.py"
fi

# Step 2: Try evo — overwrite metrics only if evo succeeds.
if ! command -v evo_ape >/dev/null 2>&1 || ! command -v evo_rpe >/dev/null 2>&1; then
  echo "INFO: evo not found; using python_fallback metrics (install: pip3 install evo)"
  {
    echo "trajectory_metrics_source=python_fallback"
    echo "evo_not_found=true"
    echo "install_hint=pip3 install evo numpy matplotlib pyyaml"
  } > "${STATUS_TXT}"
  exit 0
fi

set +e
evo_ape tum "${GT}" "${EST}" -a \
  --save_results "${OUT_DIR}/ape.zip" \
  --save_plot "${OUT_DIR}/ape.pdf" \
  > "${APE_TXT}" 2>&1
APE_STATUS=$?
evo_rpe tum "${GT}" "${EST}" -a \
  --save_results "${OUT_DIR}/rpe.zip" \
  --save_plot "${OUT_DIR}/rpe.pdf" \
  > "${RPE_TXT}" 2>&1
RPE_STATUS=$?
set -e

parse_evo_value() {
  local file="$1" key="$2"
  awk -v key="${key}" 'tolower($1) == key {print $NF; found=1} END {if (!found) print "N/A"}' "${file}"
}

if [[ "${APE_STATUS}" -eq 0 ]]; then
  ATE_RMSE="$(parse_evo_value "${APE_TXT}" rmse)"
  ATE_MEAN="$(parse_evo_value "${APE_TXT}" mean)"
  ATE_MAX="$(parse_evo_value "${APE_TXT}" max)"
  ATE_STD="$(parse_evo_value "${APE_TXT}" std)"
  RPE_RMSE="$(parse_evo_value "${RPE_TXT}" rmse)"
  RPE_MEAN="$(parse_evo_value "${RPE_TXT}" mean)"
  RPE_MAX="$(parse_evo_value "${RPE_TXT}" max)"
  RPE_STD="$(parse_evo_value "${RPE_TXT}" std)"
  {
    echo "metric,value"
    echo "ate_rmse,${ATE_RMSE}"
    echo "ate_mean,${ATE_MEAN}"
    echo "ate_max,${ATE_MAX}"
    echo "ate_std,${ATE_STD}"
    echo "rpe_rmse,${RPE_RMSE}"
    echo "rpe_mean,${RPE_MEAN}"
    echo "rpe_max,${RPE_MAX}"
    echo "rpe_std,${RPE_STD}"
    echo "trajectory_metrics_source,evo"
  } > "${METRICS_CSV}"
  echo "PASS: evo metrics written to ${METRICS_CSV}"
else
  echo "WARN: evo_ape failed (status=${APE_STATUS}); keeping python_fallback metrics"
fi

{
  echo "evo_ape_status=${APE_STATUS}"
  echo "evo_rpe_status=${RPE_STATUS}"
} > "${STATUS_TXT}"

if [[ "${APE_STATUS}" -ne 0 || "${RPE_STATUS}" -ne 0 ]]; then
  echo "WARN: evo returned non-zero. Check ${STATUS_TXT} and ${APE_TXT}."
fi
