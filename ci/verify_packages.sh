#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

expected=(
  'build-essential=12.8ubuntu1.1'
  'cmake=3.16.3-1ubuntu1.20.04.1'
  'g++=4:9.3.0-1ubuntu2'
  'g++-9=9.4.0-1ubuntu1~20.04.2'
  'gcc=4:9.3.0-1ubuntu2'
  'gcc-9=9.4.0-1ubuntu1~20.04.2'
  'git=1:2.25.1-1ubuntu3.14'
  'libc6-dev=2.31-0ubuntu9.18'
  'make=4.2.1-1.2'
  'python3=3.8.2-0ubuntu2'
  'python3.8=3.8.10-0ubuntu1~20.04.18'
)

for item in "${expected[@]}"; do
  package="${item%%=*}"
  version="${item#*=}"
  observed="$(dpkg-query -W -f='${Version}' "${package}")"
  if [[ "${observed}" != "${version}" ]]; then
    echo "package mismatch: ${package} expected ${version}, found ${observed}" >&2
    exit 1
  fi
done
