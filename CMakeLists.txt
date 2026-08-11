# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

project(wheeltec_vcu_serial
  VERSION 0.1.0
  DESCRIPTION "ROS-free Wheeltec VCU serial protocol and guarded transport"
  LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(WHEELTEC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(WHEELTEC_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(WHEELTEC_BUILD_APPS "Build command-line programs" ON)

add_library(wheeltec_vcu_serial
  src/config.cpp
  src/feedback_parser.cpp
  src/protocol.cpp
  src/safety_session.cpp
  src/transport.cpp)
add_library(wheeltec_vcu_serial::wheeltec_vcu_serial ALIAS wheeltec_vcu_serial)

target_compile_features(wheeltec_vcu_serial PUBLIC cxx_std_14)
target_include_directories(wheeltec_vcu_serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set(WHEELTEC_WARNING_FLAGS -Wall -Wextra -Wpedantic)
if(WHEELTEC_WARNINGS_AS_ERRORS)
  list(APPEND WHEELTEC_WARNING_FLAGS -Werror)
endif()
target_compile_options(wheeltec_vcu_serial PRIVATE ${WHEELTEC_WARNING_FLAGS})

if(WHEELTEC_ENABLE_SANITIZERS)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Sanitizers are supported only with GCC or Clang")
  endif()
  target_compile_options(wheeltec_vcu_serial PUBLIC
    -fno-omit-frame-pointer -fsanitize=address,undefined)
  target_link_options(wheeltec_vcu_serial PUBLIC
    -fno-omit-frame-pointer -fsanitize=address,undefined)
endif()

if(WHEELTEC_BUILD_APPS)
  add_executable(wheeltec_vcu_cli apps/wheeltec_vcu_cli.cpp)
  target_link_libraries(wheeltec_vcu_cli PRIVATE wheeltec_vcu_serial)
  target_compile_options(wheeltec_vcu_cli PRIVATE ${WHEELTEC_WARNING_FLAGS})

  add_executable(wheeltec_vcu_monitor apps/wheeltec_vcu_monitor.cpp)
  target_link_libraries(wheeltec_vcu_monitor PRIVATE wheeltec_vcu_serial)
  target_compile_options(wheeltec_vcu_monitor PRIVATE ${WHEELTEC_WARNING_FLAGS})
endif()

if(BUILD_TESTING)
  function(wheeltec_add_cpp_test target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE wheeltec_vcu_serial)
    target_compile_options(${target} PRIVATE ${WHEELTEC_WARNING_FLAGS})
    target_compile_definitions(${target} PRIVATE
      WHEELTEC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME ${target} COMMAND ${target})
    set_tests_properties(${target} PROPERTIES TIMEOUT 20)
  endfunction()

  wheeltec_add_cpp_test(test_config tests/test_config.cpp)
  wheeltec_add_cpp_test(test_protocol tests/test_protocol.cpp)
  wheeltec_add_cpp_test(test_feedback_parser tests/test_feedback_parser.cpp)
  wheeltec_add_cpp_test(test_transport tests/test_transport.cpp)
  wheeltec_add_cpp_test(test_safety_session tests/test_safety_session.cpp)
  wheeltec_add_cpp_test(test_pty_integration tests/test_pty_integration.cpp)
  target_link_libraries(test_pty_integration PRIVATE util)

  find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_teleop
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_teleop.py -v)
  set_tests_properties(test_teleop PROPERTIES
    TIMEOUT 20
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")

  if(WHEELTEC_BUILD_APPS)
    add_test(NAME test_apps_integration
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_apps_integration.py -v)
    set_tests_properties(test_apps_integration PROPERTIES
      TIMEOUT 20
      ENVIRONMENT
        "PYTHONDONTWRITEBYTECODE=1;WHEELTEC_CLI_PATH=$<TARGET_FILE:wheeltec_vcu_cli>;WHEELTEC_MONITOR_PATH=$<TARGET_FILE:wheeltec_vcu_monitor>")
  endif()
endif()

install(TARGETS wheeltec_vcu_serial
  EXPORT wheeltec_vcu_serialTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(WHEELTEC_BUILD_APPS)
  install(TARGETS wheeltec_vcu_cli wheeltec_vcu_monitor
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(PROGRAMS python/wheeltec_keyboard_teleop.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  RENAME wheeltec_keyboard_teleop)
install(FILES config/wheeltec_vcu_serial.ini
  DESTINATION ${CMAKE_INSTALL_DATADIR}/wheeltec_vcu_serial/config)
install(FILES README.md LICENSE NOTICE
  DESTINATION ${CMAKE_INSTALL_DATADIR}/wheeltec_vcu_serial)
install(DIRECTORY docs/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/wheeltec_vcu_serial/docs)

configure_package_config_file(
  cmake/wheeltec_vcu_serialConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/wheeltec_vcu_serialConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheeltec_vcu_serial)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/wheeltec_vcu_serialConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(EXPORT wheeltec_vcu_serialTargets
  FILE wheeltec_vcu_serialTargets.cmake
  NAMESPACE wheeltec_vcu_serial::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheeltec_vcu_serial)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/wheeltec_vcu_serialConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/wheeltec_vcu_serialConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheeltec_vcu_serial)
