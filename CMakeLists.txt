cmake_minimum_required(VERSION 3.8)
project(cloud_cleaner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(cloud_cleaner SHARED
  src/cloud_cleaner.cpp
)

rclcpp_components_register_nodes(cloud_cleaner
  "cloud_cleaner::PointCloudFilterNode"
)

ament_auto_add_executable(cloud_cleaner_node
    NO_TARGET_LINK_LIBRARIES
    src/cloud_cleaner_node.cpp
)

target_link_libraries(cloud_cleaner_node
  cloud_cleaner
)
include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
