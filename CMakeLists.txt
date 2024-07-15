cmake_minimum_required(VERSION 3.5)
project(raisin_vicon_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raisin_plugin REQUIRED)

set(VICON_SDK ViconDataStreamSDK_1.11.0.128037h__Linux64)
set(VICON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/${VICON_SDK})

set(VICON_LIBRARIES
  ${VICON_DIR}/libViconDataStreamSDK_CPP.so
  ${VICON_DIR}/libboost_chrono.so.1.68.0
  ${VICON_DIR}/libboost_system.so.1.68.0
  ${VICON_DIR}/libboost_thread.so.1.68.0
  ${VICON_DIR}/libboost_timer.so.1.68.0
)

add_library(${PROJECT_NAME} SHARED
  src/vicon/vicon.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${VICON_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${VICON_DIR}>
  $<INSTALL_INTERFACE:third_party/${VICON_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  raisin_plugin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY third_party/
  DESTINATION third_party
)

install(FILES
  ${VICON_LIBRARIES}
  DESTINATION lib
)

install(FILES config/params.yaml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
