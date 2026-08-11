#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mapfile -t pinned_packages < <(
  sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' \
    "${script_dir}/apt-packages.txt"
)

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends "${pinned_packages[@]}"
rm -rf /var/lib/apt/lists/*
