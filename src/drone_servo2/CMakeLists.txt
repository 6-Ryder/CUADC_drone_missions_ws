cmake_minimum_required(VERSION 3.8)
project(drone_servo2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- 舵机控制器可执行文件 ----
add_executable(servo_controller
  src/servo_controller.cpp
)

target_include_directories(servo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(servo_controller PUBLIC cxx_std_17)
target_compile_options(servo_controller PRIVATE -Wall -Wextra)

ament_target_dependencies(
  servo_controller
  rclcpp
  drone_interfaces
  px4_msgs
  std_msgs
)

# ---- 安装 ----
install(TARGETS servo_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    servo_source_contract
    test/test_servo_source_contract.py
  )
endif()

ament_package()
