#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

ci/verify_packages.sh
python3 ci/check_repository.py
python3 -m py_compile \
  python/wheeltec_keyboard_teleop.py \
  tests/test_teleop.py \
  tests/test_apps_integration.py

/usr/bin/cmake -S . -B build-ci-debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON
/usr/bin/cmake --build build-ci-debug --parallel 2
(cd build-ci-debug && /usr/bin/ctest --output-on-failure)

/usr/bin/cmake -S . -B build-ci-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON
/usr/bin/cmake --build build-ci-release --parallel 2
(cd build-ci-release && /usr/bin/ctest --output-on-failure)
/usr/bin/cmake --install build-ci-release --prefix install-ci
install-ci/bin/wheeltec_vcu_cli --help
install-ci/bin/wheeltec_vcu_monitor --help
/usr/bin/cmake -S tests/install_consumer -B build-install-consumer \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${repo_root}/install-ci"
/usr/bin/cmake --build build-install-consumer --parallel 2
build-install-consumer/install_consumer

/usr/bin/cmake -S . -B build-ci-sanitize \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON \
  -DWHEELTEC_ENABLE_SANITIZERS=ON
/usr/bin/cmake --build build-ci-sanitize --parallel 2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  /bin/bash -c 'cd build-ci-sanitize && /usr/bin/ctest --output-on-failure'

git diff --check
