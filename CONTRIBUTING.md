<!-- SPDX-License-Identifier: Apache-2.0 -->
# 贡献指南

请提交范围明确的改动，并先补充测试。协议改动必须明确区分可互操作事实与车辆特定
假设；改动应保持 wire 兼容性，否则必须引入显式版本化的 API。

提交拉取请求前，请运行：

```bash
/usr/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
/usr/bin/cmake --build build --parallel
(cd build && /usr/bin/ctest --output-on-failure)
python3 -m unittest -v tests/test_teleop.py
python3 ci/check_repository.py
git diff --check
```

车辆执行路径测试必须使用伪传输或 PTY。拉取请求不得打开物理 VCU、发送命令，
也不得包含从设备采集的数据。
