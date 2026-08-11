#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

repo_root="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
work_root="$(mktemp -d)"
trap 'rm -rf "${work_root}"' EXIT

source /opt/ros/noetic/setup.bash
# Avoid a user-local CMake shadowing Ubuntu 20.04's pinned /usr/bin/cmake.
export PATH=/usr/bin:/bin:/opt/ros/noetic/bin

core_build="${work_root}/core-build"
core_install="${work_root}/core-install"
/usr/bin/cmake -S "${repo_root}" -B "${core_build}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DWHEELTEC_BUILD_APPS=OFF \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON
/usr/bin/cmake --build "${core_build}" --parallel 2
(
  cd "${core_build}"
  /usr/bin/ctest --output-on-failure
)
/usr/bin/cmake --install "${core_build}" --prefix "${core_install}"

workspace="${work_root}/catkin-ws"
mkdir -p "${workspace}/src"
ln -s "${repo_root}/ros1/wheeltec_vcu_serial_ros1" \
  "${workspace}/src/wheeltec_vcu_serial_ros1"

(
  cd "${workspace}"
  /opt/ros/noetic/bin/catkin_make \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${core_install};/opt/ros/noetic" \
    -DCATKIN_ENABLE_TESTING=ON
  /opt/ros/noetic/bin/catkin_make run_tests
  /opt/ros/noetic/bin/catkin_test_results --verbose build/test_results
  /opt/ros/noetic/bin/catkin_make install \
    -DCMAKE_INSTALL_PREFIX="${work_root}/ros1-install"
)

installed_node="${work_root}/ros1-install/lib/wheeltec_vcu_serial_ros1/wheeltec_vcu_serial_adapter_node"
if [[ ! -x "${installed_node}" ]]; then
  echo "installed ROS 1 adapter node is missing" >&2
  exit 1
fi
if [[ -e "${work_root}/ros1-install/lib/libwheeltec_vcu_serial_ros1_support.a" ||
      -e "${work_root}/ros1-install/lib/libwheeltec_vcu_serial_ros1_support.so" ||
      -e "${work_root}/ros1-install/include/wheeltec_vcu_serial_ros1/adapter_support.hpp" ]]; then
  echo "internal adapter_support artifact leaked into the install contract" >&2
  exit 1
fi

PYTHONDONTWRITEBYTECODE=1 \
  python3 "${repo_root}/ros1/ci/check_ros1_boundary.py"
