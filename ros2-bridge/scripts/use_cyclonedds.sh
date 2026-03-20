#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script instead of executing it:" >&2
  echo "  source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${workspace_root}/install/setup.bash"
cyclone_config="${repo_root}/ros2-bridge/config/cyclonedds.xml"

if [[ ! -f "${ros_setup}" ]]; then
  echo "Missing ROS setup: ${ros_setup}" >&2
  return 1
fi

if [[ ! -f "${workspace_setup}" ]]; then
  echo "Missing workspace setup: ${workspace_setup}" >&2
  return 1
fi

if [[ ! -f "${cyclone_config}" ]]; then
  echo "Missing CycloneDDS config: ${cyclone_config}" >&2
  return 1
fi

source "${ros_setup}"
source "${workspace_setup}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${cyclone_config}"

ros2_reset_daemon() {
  command ros2 daemon stop >/dev/null 2>&1 || true
  rm -rf "${HOME}/.ros/ros2cli"
  command ros2 daemon start >/dev/null 2>&1 || true
}

_vio_ros2_should_guard() {
  case "${1:-}:${2:-}" in
    action:list|node:list|node:info|param:list|service:list|service:type|topic:find|topic:info|topic:list|topic:type)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

_vio_ros2_guarded_command() {
  local output_file
  local status

  output_file="$(mktemp)"
  if command ros2 "$@" >"${output_file}" 2>&1; then
    cat "${output_file}"
    rm -f "${output_file}"
    return 0
  fi

  status=$?
  if grep -q '!rclpy.ok()' "${output_file}"; then
    ros2_reset_daemon >/dev/null 2>&1 || true
    if command ros2 "$@" >"${output_file}" 2>&1; then
      status=0
    else
      status=$?
    fi
  fi

  cat "${output_file}"
  rm -f "${output_file}"
  return "${status}"
}

ros2() {
  if _vio_ros2_should_guard "$@"; then
    _vio_ros2_guarded_command "$@"
    return $?
  fi

  command ros2 "$@"
}

if [[ "${VIO_CYCLONEDDS_QUIET:-0}" != "1" ]]; then
  echo "ROS 2 environment ready with ${RMW_IMPLEMENTATION}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
  echo "Short ros2 CLI commands will auto-reset a stale daemon once if needed."
fi
