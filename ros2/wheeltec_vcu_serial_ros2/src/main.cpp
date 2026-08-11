// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial_ros2/adapter_node.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node =
        std::make_shared<wheeltec_vcu_serial_ros2::AdapterNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    if (!node->shutdownSafely()) {
      std::cerr << "wheeltec_vcu_serial_adapter final zero failed or was "
                   "uncertain; no VCU ACK is available\n";
      result = 9;
    }
    node.reset();
  } catch (const std::exception& error) {
    std::cerr << "wheeltec_vcu_serial_adapter startup failed: "
              << error.what() << '\n';
    result = 2;
  } catch (...) {
    std::cerr << "wheeltec_vcu_serial_adapter startup failed: unknown error\n";
    result = 3;
  }
  rclcpp::shutdown();
  return result;
}
