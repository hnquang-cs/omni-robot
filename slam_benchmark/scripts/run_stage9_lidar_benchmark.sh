#!/usr/bin/env bash

set -euo pipefail

YES=false
DRY_RUN=false
SINGLE=false
SCENARIOS=(corridor_static open_room_obstacles narrow_turn)
ALGORITHMS=(gmapping_lidar hector_lidar gmapping_stereo)
REPETITIONS=(1)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yes|-y) YES=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --single)
      SINGLE=true
      SCENARIOS=("$2")
      ALGORITHMS=("$3")
      REPETITIONS=("$4")
      shift 4
      ;;
    *)
      echo "Unknown option: $1"
      exit 2
      ;;
  esac
done

PKG_PATH="$(rospack find slam_benchmark)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
BASE="${PKG_PATH}/results/stage9"
RAW="${BASE}/raw"
MAPS="${BASE}/maps"
CSV="${BASE}/csv"
LATEX="${BASE}/latex"
MARKDOWN="${BASE}/markdown"
PLOTS="${BASE}/plots"
LOGS="${BASE}/logs"
TRAJ="${BASE}/trajectories"
mkdir -p "${RAW}" "${MAPS}" "${CSV}" "${LATEX}" "${MARKDOWN}" "${PLOTS}" "${LOGS}" "${TRAJ}"
SUMMARY="${CSV}/stage9_summary.csv"
MEAN="${CSV}/stage9_summary_mean.csv"
LATEX_TABLE="${LATEX}/table_lidar_vs_stereo.tex"
MARKDOWN_TABLE="${MARKDOWN}/table_lidar_vs_stereo.md"
LOG="${LOGS}/run_${RUN_ID}.log"

echo "Sensor Source,SLAM Algorithm,Scenario,ATE RMSE,RPE RMSE,Map Unknown Ratio,Runtime,Success,Notes" > "${SUMMARY}"

if [[ "${YES}" != true && "${SINGLE}" != true ]]; then
  read -r -p "Run Stage 9 benchmark trials now? Type YES: " answer
  [[ "${answer}" == "YES" ]] || { echo "Aborted."; exit 0; }
fi

algorithm_field() {
  case "$1" in
    gmapping_lidar) echo "lidar,Gmapping,map,/map,base_footprint,gmapping_lidar_full" ;;
    hector_lidar) echo "lidar,Hector,hector_map,/map_hector,base_footprint,hector_lidar_full" ;;
    gmapping_stereo) echo "stereo,Gmapping,map,/map,base_footprint,gmapping_stereo_full" ;;
    *) echo "unknown,unknown,map,/map,base_footprint,gmapping_lidar_full" ;;
  esac
}

run_trial() {
  local scenario="$1"
  local algorithm="$2"
  local rep="$3"
  local fields sensor slam map_frame map_topic base_frame launch_mode
  fields="$(algorithm_field "${algorithm}")"
  IFS=',' read -r sensor slam map_frame map_topic base_frame launch_spec <<< "${fields}"
  launch_mode="${launch_spec}"
  local tag="${scenario}_${algorithm}_rep${rep}_${RUN_ID}"
  local bag="${RAW}/${tag}.bag"
  local map_prefix="${MAPS}/${tag}"
  local runtime="0"
  local success="false"
  local notes="semi_auto"
  local unknown="N/A"
  local ate="N/A"
  local rpe="N/A"

  echo "Trial ${tag}" | tee -a "${LOG}"
  if [[ "${DRY_RUN}" == true ]]; then
    echo "DRY RUN: ${launch_mode}" | tee -a "${LOG}"
    echo "${sensor},${slam},${scenario},${ate},${rpe},${unknown},${runtime},${success},dry_run" >> "${SUMMARY}"
    return
  fi

  local start end
  start="$(date +%s)"
  case "${launch_mode}" in
    gmapping_lidar_full)
      roslaunch slam_benchmark slam_lidar_full.launch use_rviz:=false gazebo_gui:=false use_gmapping:=true >"${LOGS}/${tag}_slam.log" 2>&1 &
      ;;
    hector_lidar_full)
      roslaunch slam_benchmark slam_lidar_full.launch use_rviz:=false gazebo_gui:=false use_gmapping:=false >"${LOGS}/${tag}_sim.log" 2>&1 &
      local sim_pid=$!
      sleep 8
      roslaunch slam_benchmark hector_lidar.launch >"${LOGS}/${tag}_slam.log" 2>&1 &
      ;;
    gmapping_stereo_full)
      roslaunch slam_benchmark slam_sim_full.launch use_rviz:=false gazebo_gui:=false scan_topic:=/scan >"${LOGS}/${tag}_slam.log" 2>&1 &
      ;;
  esac
  local slam_pid=$!
  sleep 8
  if [[ "${sensor}" == "lidar" ]]; then
    timeout 50 rosrun slam_benchmark drive_lidar_mapping_pattern.sh corridor >>"${LOG}" 2>&1 || true
  else
    timeout 50 rosrun slam_benchmark drive_mapping_pattern.sh corridor >>"${LOG}" 2>&1 || true
  fi
  timeout 25 rosbag record -O "${bag}" /lidar/scan /scan /odom "${map_topic}" /map /tf /tf_static /clock /cmd_vel /gazebo/model_states >>"${LOG}" 2>&1 || true
  if timeout 10 rostopic echo -n 1 "${map_topic}" >/tmp/stage9_benchmark_map.txt 2>&1; then
    rosrun map_server map_saver -f "${map_prefix}" map:="${map_topic}" >>"${LOG}" 2>&1 || true
  fi
  kill "${slam_pid}" >/dev/null 2>&1 || true
  if [[ -n "${sim_pid:-}" ]]; then
    kill "${sim_pid}" >/dev/null 2>&1 || true
  fi
  wait "${slam_pid}" >/dev/null 2>&1 || true
  if [[ -n "${sim_pid:-}" ]]; then
    wait "${sim_pid}" >/dev/null 2>&1 || true
  fi
  end="$(date +%s)"
  runtime="$((end - start))"

  if [[ -s "${bag}" ]]; then
    rosrun slam_benchmark extract_gazebo_ground_truth.py "${bag}" omni_robot "${TRAJ}/${tag}_gt.tum" >>"${LOG}" 2>&1 || true
    rosrun slam_benchmark extract_slam_tf_trajectory.py "${bag}" "${map_frame}" "${base_frame}" "${TRAJ}/${tag}_slam.tum" >>"${LOG}" 2>&1 || true
    if command -v evo_ape >/dev/null 2>&1 && [[ -s "${TRAJ}/${tag}_gt.tum" && -s "${TRAJ}/${tag}_slam.tum" ]]; then
      evo_ape tum "${TRAJ}/${tag}_gt.tum" "${TRAJ}/${tag}_slam.tum" -a >"${TRAJ}/${tag}_ape.txt" 2>&1 || true
      ate="$(awk '/rmse/ {print $2; exit}' "${TRAJ}/${tag}_ape.txt" 2>/dev/null || echo N/A)"
    fi
    if command -v evo_rpe >/dev/null 2>&1 && [[ -s "${TRAJ}/${tag}_gt.tum" && -s "${TRAJ}/${tag}_slam.tum" ]]; then
      evo_rpe tum "${TRAJ}/${tag}_gt.tum" "${TRAJ}/${tag}_slam.tum" -a >"${TRAJ}/${tag}_rpe.txt" 2>&1 || true
      rpe="$(awk '/rmse/ {print $2; exit}' "${TRAJ}/${tag}_rpe.txt" 2>/dev/null || echo N/A)"
    fi
  fi

  if [[ -s "${map_prefix}.yaml" ]]; then
    rosrun slam_benchmark evaluate_map_basic.py "${map_prefix}.yaml" -o "${CSV}/${tag}_map_metrics.csv" >>"${LOG}" 2>&1 || true
    unknown="$(awk -F, 'NR==2 {for (i=1;i<=NF;i++) if (h[i]=="unknown_ratio") print $i} NR==1 {for (i=1;i<=NF;i++) h[i]=$i}' "${CSV}/${tag}_map_metrics.csv" 2>/dev/null || echo N/A)"
  fi

  if [[ -s "${bag}" && -s "${map_prefix}.yaml" ]]; then
    success="true"
    notes="bag_and_map_created"
  else
    notes="missing_bag_or_map"
  fi
  echo "${sensor},${slam},${scenario},${ate},${rpe},${unknown},${runtime},${success},${notes}" >> "${SUMMARY}"
}

for scenario in "${SCENARIOS[@]}"; do
  for algorithm in "${ALGORITHMS[@]}"; do
    for rep in "${REPETITIONS[@]}"; do
      run_trial "${scenario}" "${algorithm}" "${rep}"
    done
  done
done

awk -F, 'NR==1 {print; next} {key=$1","$2","$3; count[key]++; runtime[key]+=$7; success[key]+=$8=="true"; unknown[key]+=$6+0} END {for (k in count) print k",N/A,N/A,"(unknown[k]/count[k])","(runtime[k]/count[k])","success[k]"/"count[k]",mean"}' "${SUMMARY}" > "${MEAN}"

{
  echo "\\begin{tabular}{lllrrrrl}"
  echo "\\hline"
  echo "Sensor Source & SLAM Algorithm & Scenario & ATE RMSE & RPE RMSE & Unknown & Runtime & Success \\\\"
  echo "\\hline"
  awk -F, 'NR>1 {printf "%s & %s & %s & %s & %s & %s & %s & %s \\\\\n",$1,$2,$3,$4,$5,$6,$7,$8}' "${SUMMARY}"
  echo "\\hline"
  echo "\\end{tabular}"
} > "${LATEX_TABLE}"

{
  echo "| Sensor Source | SLAM Algorithm | Scenario | ATE RMSE | RPE RMSE | Map Unknown Ratio | Runtime | Success | Notes |"
  echo "|---|---|---|---:|---:|---:|---:|---:|---|"
  awk -F, 'NR>1 {printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s |\n",$1,$2,$3,$4,$5,$6,$7,$8,$9}' "${SUMMARY}"
} > "${MARKDOWN_TABLE}"

for name in bar_ate_lidar_vs_stereo bar_unknown_ratio_lidar_vs_stereo bar_runtime_lidar_vs_stereo; do
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$PLOTS/${name}.png" <<'PY'
import sys
from pathlib import Path
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(4, 3))
    ax.text(0.5, 0.5, "Run benchmark\nto populate plot", ha="center", va="center")
    ax.set_axis_off()
    fig.savefig(sys.argv[1], dpi=120, bbox_inches="tight")
except Exception:
    Path(sys.argv[1]).write_bytes(b"")
PY
  fi
done

echo "Summary CSV: ${SUMMARY}"
echo "Mean CSV: ${MEAN}"
echo "LaTeX table: ${LATEX_TABLE}"
echo "Markdown table: ${MARKDOWN_TABLE}"
echo "Plots: ${PLOTS}"
