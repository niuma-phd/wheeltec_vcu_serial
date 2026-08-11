<!-- SPDX-License-Identifier: Apache-2.0 -->
# Contributing

Submit focused changes with tests first. Protocol changes must distinguish
interoperability facts from vehicle-specific assumptions and preserve wire
compatibility or introduce an explicitly versioned API.

Before opening a pull request, run:

```bash
/usr/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
/usr/bin/cmake --build build --parallel
(cd build && /usr/bin/ctest --output-on-failure)
python3 -m unittest -v tests/test_teleop.py
python3 ci/check_repository.py
git diff --check
```

Use fake transports and PTYs for actuation-path testing. A pull request must not
open a physical VCU, transmit a command, or include captured device data.
