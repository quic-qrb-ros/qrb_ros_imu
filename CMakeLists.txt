cmake_minimum_required(VERSION 3.8.2)

project(qrb_ros_imu
        LANGUAGES C CXX)

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

include_directories (./include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/imu_component.cpp
  src/imu_type_adapter.cpp
)

ament_auto_add_library(${PROJECT_NAME}_test SHARED
  src/imu_test.cpp
  src/imu_type_adapter.cpp
)

ament_auto_add_executable(imu_node
  src/imu_node.cpp
)

ament_auto_add_executable(imu_test
  src/imu_test.cpp
  src/imu_type_adapter.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "qrb::ros::ImuComponent"
)

rclcpp_components_register_nodes(${PROJECT_NAME}_test
  PLUGIN "qrb::ros::ImuListener"
)

target_link_libraries(${PROJECT_NAME}
  sensor_client
)

target_link_libraries(imu_node
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/qrb_ros_imu.py)
endif()

ament_auto_package()