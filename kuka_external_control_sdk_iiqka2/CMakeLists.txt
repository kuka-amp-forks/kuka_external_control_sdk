cmake_minimum_required(VERSION 3.13)

project(kuka_external_control_sdk_iiqka2)

set(iiQKA2_directory ${CMAKE_CURRENT_SOURCE_DIR})

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "default install path" FORCE)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Threads REQUIRED)
find_package(tinyxml2 QUIET)
if(NOT tinyxml2_FOUND)
  find_package(tinyxml2_vendor REQUIRED)
endif()
find_package(kuka_external_control_sdk_common QUIET)
if(NOT kuka_external_control_sdk_common_FOUND)
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")
  find_package(kuka_external_control_sdk_common REQUIRED)
endif()

set(common_target_name "Kuka::kuka-external-control-sdk-common")
set(target_name "kuka-external-control-sdk-iiqka2")

set(sources
  ${iiQKA2_directory}/src/eki/client.cc
  ${iiQKA2_directory}/src/eki/robot_interface.cc
  ${iiQKA2_directory}/src/mxa/client.cc
  ${iiQKA2_directory}/src/mxa/robot_interface.cc
  ${iiQKA2_directory}/src/rsi/endpoint.cc
  ${iiQKA2_directory}/src/rsi/robot_interface.cc
  ${iiQKA2_directory}/src/message_builder.cc
  ${iiQKA2_directory}/src/robot.cc
)

add_library(${target_name} STATIC ${sources})

target_include_directories(${target_name} PUBLIC
  "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${target_name}
  PUBLIC
    Threads::Threads
    ${common_target_name}
  PRIVATE
    tinyxml2
)

get_target_property(COMMON_INCLUDE_DIRS ${common_target_name} INTERFACE_INCLUDE_DIRECTORIES)
if(COMMON_INCLUDE_DIRS)
  target_include_directories(${target_name} PUBLIC ${COMMON_INCLUDE_DIRS})
endif()

target_compile_features(${target_name} INTERFACE cxx_std_17 PRIVATE cxx_std_17)

if(BUILD_TESTING)
  find_package(GTest)
  enable_testing()

  add_executable(
    test_iiqka2_message_builder
    test/test_message_builder.cc
  )

  add_executable(
    test_eki_client_parsing
    test/test_eki_client_parsing.cc
  )

  add_executable(
    test_mxa_rsi_start
    test/test_mxa_rsi_start.cc
  )

  add_executable(
    test_mxa_telegram_hook
    test/test_mxa_telegram_hook.cc
  )

  add_executable(
    test_status_update
    test/test_status_update.cc
  )

  add_executable(
    test_mxa_brake_test_status
    test/test_mxa_brake_test_status.cc
  )

  target_link_libraries(
    test_iiqka2_message_builder
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_link_libraries(
    test_eki_client_parsing
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_link_libraries(
    test_mxa_rsi_start
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_link_libraries(
    test_mxa_telegram_hook
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_link_libraries(
    test_status_update
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_link_libraries(
    test_mxa_brake_test_status
    GTest::gtest_main
    ${target_name}
    GTest::gtest
  )

  target_include_directories(test_iiqka2_message_builder PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
    "$<BUILD_INTERFACE:${iiQKA2_directory}/test>"
  )

  target_include_directories(test_eki_client_parsing PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
    "$<BUILD_INTERFACE:${iiQKA2_directory}/test>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../kuka_external_control_sdk_common/utils/include>"
  )

  target_include_directories(test_mxa_rsi_start PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../kuka_external_control_sdk_common/utils/include>"
  )

  target_include_directories(test_mxa_telegram_hook PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../kuka_external_control_sdk_common/utils/include>"
  )

  target_include_directories(test_status_update PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
  )

  target_include_directories(test_mxa_brake_test_status PUBLIC
    "$<BUILD_INTERFACE:${iiQKA2_directory}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../kuka_external_control_sdk_common/utils/include>"
  )

  target_compile_features(test_iiqka2_message_builder INTERFACE cxx_std_17 PRIVATE cxx_std_17)
  target_compile_features(test_eki_client_parsing INTERFACE cxx_std_17 PRIVATE cxx_std_17)
  target_compile_features(test_mxa_rsi_start INTERFACE cxx_std_17 PRIVATE cxx_std_17)
  target_compile_features(test_mxa_telegram_hook INTERFACE cxx_std_17 PRIVATE cxx_std_17)
  target_compile_features(test_status_update INTERFACE cxx_std_17 PRIVATE cxx_std_17)
  target_compile_features(test_mxa_brake_test_status INTERFACE cxx_std_17 PRIVATE cxx_std_17)

  include(GoogleTest)
  gtest_discover_tests(test_iiqka2_message_builder)
  gtest_discover_tests(test_eki_client_parsing)
  gtest_discover_tests(test_status_update)
  # Both binaries can start a real Client bound to the mxA client's fixed send
  # port, so they share a lock to keep ctest from running them at once
  gtest_discover_tests(test_mxa_rsi_start PROPERTIES RESOURCE_LOCK mxa_udp_send_port)
  gtest_discover_tests(test_mxa_telegram_hook PROPERTIES RESOURCE_LOCK mxa_udp_send_port)
  gtest_discover_tests(test_mxa_brake_test_status PROPERTIES RESOURCE_LOCK mxa_udp_send_port)
endif()

install(TARGETS ${target_name} EXPORT SdkExport
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(EXPORT SdkExport NAMESPACE Kuka::
    FILE kuka-external-control-sdk-iiqka2Targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kuka-external-control-sdk-iiqka2
)

install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/cmake/kuka-external-control-sdk-iiqka2Config.cmake
    ${CMAKE_CURRENT_LIST_DIR}/cmake/Findtinyxml2.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kuka-external-control-sdk-iiqka2
)

install(DIRECTORY ${iiQKA2_directory}/include/
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include
      FILES_MATCHING PATTERN "*.h" PATTERN "*.hh")

# Install package.xml file so this package can be processed by ROS toolings
# See REP 136 for details
# Installing this in non-ROS environments won't have any effect, but it won't harm, either.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Generates the colcon environment hooks needed to put the package on AMENT_PREFIX_PATH
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_targets(SdkExport HAS_LIBRARY_TARGET NAMESPACE Kuka::)
  ament_export_dependencies(Threads kuka_external_control_sdk_common)
  ament_package()
else()
#### Begin import ####
# Imported from ros-industrial/ros_industrial_cmake_boilerplate
# Copyright (C) 2018 by George Cave - gcave@stablecoder.ca
# Copyright (c) 2020, Southwest Research Institute
# Licensed under Apache-2.0 license

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME}
    DESTINATION share/ament_index/resource_index/packages
)
file(WRITE
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv
    "prepend-non-duplicate;AMENT_PREFIX_PATH;"
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv
    DESTINATION share/${PROJECT_NAME}/hook
)
file(WRITE
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv
    "prepend-non-duplicate;ROS_PACKAGE_PATH;"
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv
    DESTINATION share/${PROJECT_NAME}/hook
)
#### End import ####
endif()
