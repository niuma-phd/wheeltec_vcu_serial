#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail

source /opt/ros/humble/setup.bash

readonly source_root="${WHEELTEC_SOURCE_ROOT:-/workspace}"
readonly temporary_root="$(mktemp -d /tmp/wheeltec_ros2_ci.XXXXXX)"
trap 'rm -rf -- "${temporary_root}"' EXIT

cmake -S "${source_root}" -B "${temporary_root}/core-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DWHEELTEC_BUILD_APPS=OFF \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON \
  -DCMAKE_INSTALL_PREFIX="${temporary_root}/core-install"
cmake --build "${temporary_root}/core-build" --parallel 2
cmake --install "${temporary_root}/core-build"

export CMAKE_PREFIX_PATH="${temporary_root}/core-install:${CMAKE_PREFIX_PATH:-}"
colcon --log-base "${temporary_root}/log" build \
  --base-paths "${source_root}/ros2/wheeltec_vcu_serial_ros2" \
  --build-base "${temporary_root}/build" \
  --install-base "${temporary_root}/install" \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DBUILD_TESTING=ON

source "${temporary_root}/install/local_setup.bash"

readonly ctest_list="${temporary_root}/ctest-list.txt"
(
  cd "${temporary_root}/build/wheeltec_vcu_serial_ros2"
  ctest -N
) >"${ctest_list}"
for expected_test in test_adapter_logic test_contract lint_cmake xmllint; do
  if ! grep -E "Test +#[0-9]+: ${expected_test}$" "${ctest_list}" >/dev/null; then
    cat "${ctest_list}"
    echo "required ROS 2 test was not registered: ${expected_test}" >&2
    exit 1
  fi
done

colcon --log-base "${temporary_root}/test-log" test \
  --base-paths "${source_root}/ros2/wheeltec_vcu_serial_ros2" \
  --build-base "${temporary_root}/build" \
  --install-base "${temporary_root}/install" \
  --event-handlers console_direct+
colcon --log-base "${temporary_root}/result-log" test-result \
  --test-result-base "${temporary_root}/build" --verbose

ros2 interface show wheeltec_vcu_serial_ros2/msg/DriveCommand \
  | grep -F "builtin_interfaces/Duration valid_for"
ros2 interface show wheeltec_vcu_serial_ros2/msg/Feedback \
  | grep -F "uint8 composite_stop_flag_raw"
ros2 interface show wheeltec_vcu_serial_ros2/msg/Feedback \
  | grep -F "bool vcu_ack_available"
ros2 interface show wheeltec_vcu_serial_ros2/msg/AdapterState \
  | grep -F "uint64 connection_generation"
ros2 interface show wheeltec_vcu_serial_ros2/msg/AdapterState \
  | grep -F "bool source_time_available"
ros2 pkg executables wheeltec_vcu_serial_ros2 \
  | grep -F "wheeltec_vcu_serial_adapter"

PYTHONPYCACHEPREFIX="${temporary_root}/pycache" python3 -m compileall -q \
  "${source_root}/ros2/wheeltec_vcu_serial_ros2/launch" \
  "${source_root}/ros2/wheeltec_vcu_serial_ros2/test"
python3 "${source_root}/ci/check_repository.py"

readonly launch_log="${temporary_root}/offline-launch.log"
set +e
ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=2s 3s \
  ros2 launch wheeltec_vcu_serial_ros2 wheeltec_vcu_serial.launch.py \
  >"${launch_log}" 2>&1
readonly launch_status=$?
set -e
if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 124 ]]; then
  cat "${launch_log}"
  exit "${launch_status}"
fi
grep -F "adapter is offline" "${launch_log}"
if grep -F "serial connection opened" "${launch_log}"; then
  echo "offline launch unexpectedly opened a serial connection" >&2
  exit 1
fi

echo "ROS 2 Humble adapter CI passed without a mapped hardware device"
