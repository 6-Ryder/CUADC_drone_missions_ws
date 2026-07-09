cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 消息定义
set(msg_files
  msg/CylinderDetection.msg
  msg/CylinderArray.msg
  msg/HazardDetection.msg
  msg/HazardArray.msg
  msg/LandingTarget.msg
  msg/CoverageStatus.msg
)

# 服务定义
set(srv_files
  srv/TriggerDrop.srv
  srv/SetVisionMode.srv
)

# 动作定义
set(action_files
  action/PrecisionLand.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
