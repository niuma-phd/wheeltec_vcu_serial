# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_vcu_serial_ros1 VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_srvs)
find_package(wheeltec_vcu_serial 0.1 CONFIG REQUIRED)

add_message_files(FILES
  AdapterState.msg
  DriveCommand.msg
  Feedback.msg)
add_service_files(FILES
  Authorize.srv
  ResetEstop.srv)
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_srvs)

set(WHEELTEC_ROS1_WARNING_FLAGS -Wall -Wextra -Wpedantic -Werror)

add_library(${PROJECT_NAME}_support STATIC
  src/adapter_support.cpp)
target_compile_features(${PROJECT_NAME}_support PUBLIC cxx_std_14)
target_compile_options(${PROJECT_NAME}_support PRIVATE
  ${WHEELTEC_ROS1_WARNING_FLAGS})
target_include_directories(${PROJECT_NAME}_support
  PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}_support
  PUBLIC
    ${catkin_LIBRARIES}
    wheeltec_vcu_serial::wheeltec_vcu_serial)
add_dependencies(${PROJECT_NAME}_support
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(wheeltec_vcu_serial_adapter_node
  src/adapter_node.cpp)
target_compile_features(wheeltec_vcu_serial_adapter_node PRIVATE cxx_std_14)
target_compile_options(wheeltec_vcu_serial_adapter_node PRIVATE
  ${WHEELTEC_ROS1_WARNING_FLAGS})
target_link_libraries(wheeltec_vcu_serial_adapter_node
  PRIVATE
    ${PROJECT_NAME}_support)
add_dependencies(wheeltec_vcu_serial_adapter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_adapter_support
    test/test_adapter_support.cpp)
  if(TARGET test_adapter_support)
    target_compile_features(test_adapter_support PRIVATE cxx_std_14)
    target_compile_options(test_adapter_support PRIVATE
      ${WHEELTEC_ROS1_WARNING_FLAGS})
    target_compile_definitions(test_adapter_support PRIVATE
      WHEELTEC_ROS1_PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_adapter_support
      ${PROJECT_NAME}_support)
    add_dependencies(test_adapter_support
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS})
  endif()
  add_rostest(test/offline_node.test)
endif()

install(TARGETS wheeltec_vcu_serial_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
