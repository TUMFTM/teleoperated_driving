#!/usr/bin/env bash
set -uo pipefail

ulimit -c 0

label="${1:-colcon}"
if [[ $# -gt 0 ]]; then
  shift
fi

max_attempts="${COLCON_RETRY_MAX:-5}"
parallel_workers="${COLCON_PARALLEL_WORKERS:-2}"
cmake_parallel_level="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"

clean_failed_packages() {
  local log_file="$1"
  local packages
  packages="$(awk '
    /(Failed|Aborted)[[:space:]]+<<</ {
      for (i = 1; i <= NF; i++) {
        if ($i == "<<<" && (i + 1) <= NF) {
          print $(i + 1)
        }
      }
    }
  ' "$log_file" | sort -u)"

  if [[ -z "$packages" ]]; then
    echo "No failed package names found; cleaning build directory to avoid stale generated files."
    rm -rf build log
    return
  fi

  for package in $packages; do
    echo "Cleaning failed package state for ${package}"
    rm -rf "build/${package}" "install/${package}" "log/latest_build/${package}"
    find log -maxdepth 2 -type d -name "${package}" -exec rm -rf {} + 2>/dev/null || true
  done
}

attempt=1
while [[ "$attempt" -le "$max_attempts" ]]; do
  log_file="/tmp/${label}-colcon-attempt-${attempt}.log"
  echo "== ${label} colcon attempt ${attempt}/${max_attempts} =="

  CMAKE_BUILD_PARALLEL_LEVEL="$cmake_parallel_level" \
    colcon build --parallel-workers "$parallel_workers" "$@" 2>&1 | tee "$log_file"
  status="${PIPESTATUS[0]}"

  if [[ "$status" -eq 0 ]]; then
    exit 0
  fi

  echo "${label} colcon build failed on attempt ${attempt} with status ${status}"
  if [[ "$attempt" -eq "$max_attempts" ]]; then
    exit "$status"
  fi

  clean_failed_packages "$log_file"
  attempt="$((attempt + 1))"
  sleep 3
done

exit 1
