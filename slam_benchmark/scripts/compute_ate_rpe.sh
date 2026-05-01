#!/usr/bin/env bash

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
  } > "${METRICS_CSV}"
  {
    echo "FAIL: ${reason}"
    echo "Install suggestion: pip3 install evo numpy matplotlib pyyaml"
    echo "Manual commands:"
    echo "  evo_ape tum ${GT} ${EST} -a --save_results ${OUT_DIR}/ape.zip --save_plot ${OUT_DIR}/ape.pdf"
    echo "  evo_rpe tum ${GT} ${EST} -a --save_results ${OUT_DIR}/rpe.zip --save_plot ${OUT_DIR}/rpe.pdf"
  } > "${STATUS_TXT}"
}

if [[ ! -s "${GT}" ]]; then
  write_na_metrics "ground-truth trajectory is missing or empty: ${GT}"
  cat "${STATUS_TXT}"
  exit 0
fi

if [[ ! -s "${EST}" ]]; then
  write_na_metrics "estimated trajectory is missing or empty: ${EST}"
  cat "${STATUS_TXT}"
  exit 0
fi

if ! command -v evo_ape >/dev/null 2>&1 || ! command -v evo_rpe >/dev/null 2>&1; then
  write_na_metrics "evo_ape/evo_rpe not found"
  cat "${STATUS_TXT}"
  exit 0
fi

set +e
evo_ape tum "${GT}" "${EST}" -a --save_results "${OUT_DIR}/ape.zip" --save_plot "${OUT_DIR}/ape.pdf" > "${APE_TXT}" 2>&1
APE_STATUS=$?
evo_rpe tum "${GT}" "${EST}" -a --save_results "${OUT_DIR}/rpe.zip" --save_plot "${OUT_DIR}/rpe.pdf" > "${RPE_TXT}" 2>&1
RPE_STATUS=$?
set -e

parse_value() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" 'tolower($1) == key {print $NF; found=1} END {if (!found) print "N/A"}' "${file}"
}

ATE_RMSE="$(parse_value "${APE_TXT}" rmse)"
ATE_MEAN="$(parse_value "${APE_TXT}" mean)"
ATE_MAX="$(parse_value "${APE_TXT}" max)"
ATE_STD="$(parse_value "${APE_TXT}" std)"
RPE_RMSE="$(parse_value "${RPE_TXT}" rmse)"
RPE_MEAN="$(parse_value "${RPE_TXT}" mean)"
RPE_MAX="$(parse_value "${RPE_TXT}" max)"
RPE_STD="$(parse_value "${RPE_TXT}" std)"

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
} > "${METRICS_CSV}"

{
  echo "evo_ape_status=${APE_STATUS}"
  echo "evo_rpe_status=${RPE_STATUS}"
  echo "ape_output=${APE_TXT}"
  echo "rpe_output=${RPE_TXT}"
} > "${STATUS_TXT}"

echo "PASS: wrote ${METRICS_CSV}"
if [[ "${APE_STATUS}" -ne 0 || "${RPE_STATUS}" -ne 0 ]]; then
  echo "WARN: evo returned non-zero status. Check ${STATUS_TXT} and raw output files."
fi
