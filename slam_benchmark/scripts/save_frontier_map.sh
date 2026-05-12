#!/usr/bin/env bash
# Stage 11: save the current Gmapping /map to results/stage11_frontier/maps/.
#
# Usage: save_frontier_map.sh [basename]
#   basename defaults to frontier_gmapping_<timestamp>
# Always also writes frontier_gmapping_latest.{yaml,pgm} as a copy.

set -uo pipefail

PKG_PATH="$(rospack find slam_benchmark 2>/dev/null || true)"
if [[ -z "${PKG_PATH}" ]]; then
  PKG_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
MAP_DIR="${PKG_PATH}/results/stage11_frontier/maps"
LOG_DIR="${PKG_PATH}/results/stage11_frontier/logs"
mkdir -p "${MAP_DIR}" "${LOG_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BASENAME="${1:-frontier_gmapping_${STAMP}}"
PREFIX="${MAP_DIR}/${BASENAME}"
LATEST_PREFIX="${MAP_DIR}/frontier_gmapping_latest"
LOG_PATH="${LOG_DIR}/${BASENAME}_save.log"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL  ROS master is not reachable" | tee "${LOG_PATH}"
  exit 1
fi

if ! rostopic list 2>/dev/null | grep -Fxq "/map"; then
  echo "FAIL  /map topic not advertised. Is Gmapping running?" | tee "${LOG_PATH}"
  exit 1
fi

if ! timeout 10 rostopic echo -n 1 /map >/dev/null 2>&1; then
  echo "FAIL  /map advertised but no message received in 10s" | tee "${LOG_PATH}"
  exit 1
fi

echo "Saving map to ${PREFIX}.{yaml,pgm}" | tee -a "${LOG_PATH}"
if ! rosrun map_server map_saver -f "${PREFIX}" >>"${LOG_PATH}" 2>&1; then
  echo "FAIL  map_saver failed. See ${LOG_PATH}" | tee -a "${LOG_PATH}"
  exit 1
fi

if [[ ! -s "${PREFIX}.yaml" || ! -s "${PREFIX}.pgm" ]]; then
  echo "FAIL  expected files not produced at ${PREFIX}" | tee -a "${LOG_PATH}"
  exit 1
fi

cp -f "${PREFIX}.yaml" "${LATEST_PREFIX}.yaml"
cp -f "${PREFIX}.pgm"  "${LATEST_PREFIX}.pgm"

echo "PASS  saved ${PREFIX}.{yaml,pgm}" | tee -a "${LOG_PATH}"
echo "PASS  refreshed ${LATEST_PREFIX}.{yaml,pgm}" | tee -a "${LOG_PATH}"
echo "${PREFIX}.yaml"
exit 0
