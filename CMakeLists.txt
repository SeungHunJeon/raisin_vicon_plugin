cmake_minimum_required(VERSION 3.5)
project(raisin_vicon_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(raisin_plugin REQUIRED)
find_package(raisin_interfaces REQUIRED)
find_package(raisin_data_logger REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(raisim REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/vicon/vicon.cpp
)

target_link_libraries(${PROJECT_NAME} raisim::raisim)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  raisin_plugin
  geometry_msgs
  raisin_interfaces
  raisin_data_logger
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs)

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

install(FILES
  ${VICON_LIBRARIES}
  DESTINATION lib
)

install(FILES config/params.yaml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
