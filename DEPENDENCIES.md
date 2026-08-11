<!-- SPDX-License-Identifier: Apache-2.0 -->
# Dependency register

The runtime library depends only on the C++14 standard library and Linux/POSIX
APIs. The keyboard program uses only the Python 3.8 standard library. ROS is not
a dependency of `main`.

Target build environment:

| Component | Pinned Ubuntu 20.04 package/version | Purpose |
| --- | --- | --- |
| `build-essential` | `12.8ubuntu1.1` | Compiler and base build tools |
| `gcc`, `g++` | `4:9.3.0-1ubuntu2` | GCC 9 toolchain selectors |
| `gcc-9`, `g++-9` | `9.4.0-1ubuntu1~20.04.2` | C++14 compiler |
| `cmake` | `3.16.3-1ubuntu1.20.04.1` | Build system |
| `make` | `4.2.1-1.2` | Build executor |
| `git` | `1:2.25.1-1ubuntu3.14` | Repository validation in CI |
| `libc6-dev` | `2.31-0ubuntu9.18` | POSIX/Linux development headers |
| `python3` | `3.8.2-0ubuntu2` | Python interpreter selector |
| `python3.8` | `3.8.10-0ubuntu1~20.04.18` | Keyboard program and tests |

The CI build container is
`ubuntu:20.04@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214`.
The digest identifies the multi-platform image manifest; CI runs its amd64
variant. Package versions are checked with `dpkg-query` before compilation.

GitHub Actions:

| Action | Immutable revision | Use |
| --- | --- | --- |
| `actions/checkout` | `11bd71901bbe5b1630ceea73d27597364c9af683` | Source checkout; credentials are not persisted |

No action or package from this table is redistributed in the repository. Their
licenses remain those supplied by their respective projects and Ubuntu.
