# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(wheeltec_vcu_serial_install_consumer LANGUAGES CXX)

find_package(wheeltec_vcu_serial 0.1 CONFIG REQUIRED)

add_executable(install_consumer main.cpp)
target_link_libraries(install_consumer
  PRIVATE wheeltec_vcu_serial::wheeltec_vcu_serial)
target_compile_features(install_consumer PRIVATE cxx_std_14)
target_compile_options(install_consumer PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
