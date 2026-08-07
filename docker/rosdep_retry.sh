#!/usr/bin/env bash
set -uo pipefail

ulimit -c 0

ros_distro="${1:?usage: rosdep_retry <ros-distro>}"
max_attempts="${ROSDEP_RETRY_MAX:-5}"
skip_keys="autoware_auto_planning_msgs autoware_auto_vehicle_msgs tier4_external_api_msgs"

attempt=1
while [[ "$attempt" -le "$max_attempts" ]]; do
  echo "== rosdep attempt ${attempt}/${max_attempts} =="
  rosdep update && rosdep install --from-paths src --ignore-src \
    --rosdistro "$ros_distro" --skip-keys "$skip_keys" -y
  status=$?

  if [[ "$status" -eq 0 ]]; then
    exit 0
  fi

  echo "rosdep failed on attempt ${attempt} with status ${status}"
  if [[ "$attempt" -eq "$max_attempts" ]]; then
    exit "$status"
  fi

  rm -rf "$HOME/.ros/rosdep/sources.cache" /root/.ros/rosdep/sources.cache 2>/dev/null || true
  attempt="$((attempt + 1))"
  sleep 3
done

exit 1
