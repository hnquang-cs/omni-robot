#!/usr/bin/env bash
# debug_discrete45_odom.sh
#
# Quick health-check for the discrete_45 demo. Verifies that /odom, TF,
# /cmd_vel, and /motion_primitive_cmd are wired up so the controller can
# run end-to-end. Each step uses `timeout` so the script never hangs.

set -u

ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
PRIMITIVE_TOPIC="${PRIMITIVE_TOPIC:-/motion_primitive_cmd}"
STATE_TOPIC="${STATE_TOPIC:-/motion_primitive_state}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME_PRIMARY="${BASE_FRAME:-base_footprint}"
BASE_FRAME_FALLBACK="${BASE_FRAME_FALLBACK:-base_link}"

green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[1;33m%s\033[0m\n' "$*"; }

declare -A RESULT
mark_pass() { RESULT["$1"]=PASS; green "PASS: $1"; }
mark_fail() { RESULT["$1"]=FAIL; red   "FAIL: $1${2:+ ($2)}"; }
mark_warn() { RESULT["$1"]=WARN; yellow "WARN: $1${2:+ ($2)}"; }

# ----------------------------------------------------------------------
# 1) odom_topic_exists
# ----------------------------------------------------------------------
echo "=== 1) odom topic listed ==="
if rostopic list 2>/dev/null | grep -qx "$ODOM_TOPIC"; then
  mark_pass odom_topic_exists
else
  mark_fail odom_topic_exists "$ODOM_TOPIC not found in rostopic list"
fi

# ----------------------------------------------------------------------
# 2) odom_has_message
# ----------------------------------------------------------------------
echo "=== 2) odom has at least one message ==="
if timeout 5 rostopic echo -n 1 "$ODOM_TOPIC" >/dev/null 2>&1; then
  mark_pass odom_has_message
else
  mark_fail odom_has_message "no message received within 5s on $ODOM_TOPIC"
fi

# ----------------------------------------------------------------------
# 3) odom_rate_ok
# ----------------------------------------------------------------------
echo "=== 3) odom rate ==="
HZ_OUT="$(timeout 5 rostopic hz "$ODOM_TOPIC" 2>&1 | head -n 20 || true)"
if echo "$HZ_OUT" | grep -qE "average rate:[[:space:]]*[0-9]+\\.[0-9]+"; then
  RATE=$(echo "$HZ_OUT" | grep -oE "average rate:[[:space:]]*[0-9]+\\.[0-9]+" | head -n1 | awk '{print $3}')
  echo "    measured rate: ${RATE} Hz"
  AWK_OK=$(awk -v r="$RATE" 'BEGIN{print (r >= 5.0) ? 1 : 0}')
  if [ "$AWK_OK" = "1" ]; then
    mark_pass odom_rate_ok
  else
    mark_warn odom_rate_ok "rate ${RATE} Hz < 5 Hz"
  fi
else
  mark_fail odom_rate_ok "could not measure rate"
fi

# ----------------------------------------------------------------------
# 4) tf_odom_base_ok
# ----------------------------------------------------------------------
echo "=== 4) tf_echo $ODOM_FRAME -> base ==="
TF_OK=0
TF_TARGET="$BASE_FRAME_PRIMARY"
TF_OUT="$(timeout 4 rosrun tf tf_echo "$ODOM_FRAME" "$BASE_FRAME_PRIMARY" 2>&1 || true)"
if echo "$TF_OUT" | grep -q "Translation:"; then
  TF_OK=1
else
  TF_OUT2="$(timeout 4 rosrun tf tf_echo "$ODOM_FRAME" "$BASE_FRAME_FALLBACK" 2>&1 || true)"
  if echo "$TF_OUT2" | grep -q "Translation:"; then
    TF_OK=1
    TF_TARGET="$BASE_FRAME_FALLBACK"
  fi
fi
if [ "$TF_OK" = "1" ]; then
  mark_pass tf_odom_base_ok
  echo "    TF target: $ODOM_FRAME -> $TF_TARGET"
else
  mark_fail tf_odom_base_ok "no TF $ODOM_FRAME -> {$BASE_FRAME_PRIMARY,$BASE_FRAME_FALLBACK}"
fi

# ----------------------------------------------------------------------
# 5) cmd_vel_has_subscriber
# ----------------------------------------------------------------------
echo "=== 5) /cmd_vel info ==="
CMD_INFO="$(timeout 3 rostopic info "$CMD_VEL_TOPIC" 2>&1 || true)"
echo "$CMD_INFO"
SUBS=$(echo "$CMD_INFO" | awk '/Subscribers:/{flag=1; next} /Publishers:/{flag=0} flag && /\* /')
if [ -n "$SUBS" ]; then
  mark_pass cmd_vel_has_subscriber
else
  mark_fail cmd_vel_has_subscriber "no subscriber on $CMD_VEL_TOPIC (gazebo_model_cmd_vel?)"
fi

# ----------------------------------------------------------------------
# 6) primitive_topic_ok
# ----------------------------------------------------------------------
echo "=== 6) /motion_primitive_cmd info ==="
PRIM_INFO="$(timeout 3 rostopic info "$PRIMITIVE_TOPIC" 2>&1 || true)"
echo "$PRIM_INFO"
if echo "$PRIM_INFO" | grep -q "Type: std_msgs/String"; then
  mark_pass primitive_topic_ok
else
  mark_fail primitive_topic_ok "$PRIMITIVE_TOPIC missing or wrong type"
fi

# ----------------------------------------------------------------------
# 7) state_topic info (informational)
# ----------------------------------------------------------------------
echo "=== 7) /motion_primitive_state ==="
if rostopic list 2>/dev/null | grep -qx "$STATE_TOPIC"; then
  STATE_MSG="$(timeout 3 rostopic echo -n 1 "$STATE_TOPIC" 2>&1 || true)"
  echo "$STATE_MSG"
fi

# ----------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------
echo
echo "=== Summary ==="
EXIT=0
for key in odom_topic_exists odom_has_message odom_rate_ok tf_odom_base_ok cmd_vel_has_subscriber primitive_topic_ok; do
  status="${RESULT[$key]:-UNKNOWN}"
  echo "  $key: $status"
  if [ "$status" = "FAIL" ]; then
    EXIT=1
  fi
done
exit "$EXIT"
