// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/protocol.hpp"

#include <cstddef>

int main() {
  const auto frame = wheeltec_vcu_serial::makeZeroCommandFrame();
  return frame.size() == wheeltec_vcu_serial::kCommandFrameSize &&
                 frame.front() == 0x7BU && frame.back() == 0x7DU
             ? 0
             : 1;
}
