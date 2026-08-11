# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

project(wheeltec_vcu_serial_ros2 VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wheeltec_vcu_serial 0.1 CONFIG REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdapterState.msg"
  "msg/DriveCommand.msg"
  "msg/Feedback.msg"
  "srv/Authorize.srv"
  "srv/ResetEstop.srv"
  DEPENDENCIES builtin_interfaces)

add_library(adapter_logic STATIC src/adapter_logic.cpp)
target_include_directories(adapter_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(adapter_logic PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(adapter_logic PUBLIC
  wheeltec_vcu_serial::wheeltec_vcu_serial)

add_executable(wheeltec_vcu_serial_adapter
  src/adapter_node.cpp
  src/main.cpp)
target_include_directories(wheeltec_vcu_serial_adapter PRIVATE include)
target_compile_options(wheeltec_vcu_serial_adapter PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(wheeltec_vcu_serial_adapter
  adapter_logic
  wheeltec_vcu_serial::wheeltec_vcu_serial)
ament_target_dependencies(wheeltec_vcu_serial_adapter
  builtin_interfaces
  rclcpp
  std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wheeltec_vcu_serial_adapter "${cpp_typesupport_target}")

install(TARGETS wheeltec_vcu_serial_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_cmake()
  ament_xmllint()

  ament_add_gtest(test_adapter_logic test/test_adapter_logic.cpp)
  if(TARGET test_adapter_logic)
    target_link_libraries(test_adapter_logic adapter_logic)
    target_compile_options(test_adapter_logic PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(test_contract PROPERTIES TIMEOUT 10)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
