#!/usr/bin/env bash
# Stage 11 dependency check for frontier-based exploration.
# Verifies that all packages required to run explore_lite + Gmapping + move_base
# are reachable through rospack. Does NOT install anything automatically.

set -uo pipefail

PKG_PATH="$(rospack find slam_benchmark 2>/dev/null || true)"
if [[ -z "${PKG_PATH}" ]]; then
  PKG_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

LOG_DIR="${PKG_PATH}/results/stage11_frontier/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/frontier_dependency_check.log"

REQUIRED_PKGS=(move_base gmapping map_server costmap_2d actionlib move_base_msgs)
OPTIONAL_PKGS=(explore_lite)

EXIT_CODE=0
EXPLORE_AVAILABLE=0

{
  echo "=== Stage 11 Frontier Dependency Check ==="
  echo "Timestamp: $(date -Iseconds)"
  echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
  echo

  echo "-- Required packages --"
  for pkg in "${REQUIRED_PKGS[@]}"; do
    path="$(rospack find "${pkg}" 2>/dev/null || true)"
    if [[ -n "${path}" ]]; then
      printf "PASS  %-20s %s\n" "${pkg}" "${path}"
    else
      printf "FAIL  %-20s NOT FOUND\n" "${pkg}"
      EXIT_CODE=1
    fi
  done

  echo
  echo "-- Optional / preferred packages --"
  for pkg in "${OPTIONAL_PKGS[@]}"; do
    path="$(rospack find "${pkg}" 2>/dev/null || true)"
    if [[ -n "${path}" ]]; then
      printf "PASS  %-20s %s\n" "${pkg}" "${path}"
      if [[ "${pkg}" == "explore_lite" ]]; then
        EXPLORE_AVAILABLE=1
      fi
    else
      printf "WARN  %-20s NOT FOUND\n" "${pkg}"
    fi
  done

  echo
  if [[ "${EXPLORE_AVAILABLE}" -eq 1 ]]; then
    echo "explore_lite_available"
  else
    echo "explore_lite_missing"
    cat <<'EOF'

>>> explore_lite is missing. Suggested installation paths (run manually):

Option A: install binary
  sudo apt update
  sudo apt install ros-noetic-explore-lite
  # or, depending on your apt index:
  sudo apt install ros-noetic-m-explore

Option B: build from source
  cd ~/catkin_ws/src
  git clone https://github.com/hrnr/m-explore.git
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash

Do NOT auto-install. After install, re-run this script to confirm.
If explore_lite remains unavailable, the fallback simple_frontier_detector.py
can be used (see slam_benchmark/scripts/simple_frontier_detector.py).
EOF
  fi

  echo
  echo "-- Required-package status: ${EXIT_CODE} (0 = all required present) --"
  echo "-- explore_lite status:    $([[ ${EXPLORE_AVAILABLE} -eq 1 ]] && echo present || echo missing) --"
} 2>&1 | tee "${LOG_FILE}"

# Exit non-zero only if required packages are missing.
exit "${EXIT_CODE}"
