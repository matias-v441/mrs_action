cmake_minimum_required(VERSION 3.5)
project(mrs_action)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

### CPP lib ###
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mrs_action_msgs REQUIRED)

add_library(action_manager_lib INTERFACE)

target_include_directories(action_manager_lib INTERFACE src/include)

ament_target_dependencies(action_manager_lib INTERFACE
  rclcpp
  rclcpp_action
  mrs_action_msgs
  mrs_msgs
)

### Node ###

add_executable(mrs_action_server
 src/action_manager_node.cpp
 )
target_link_libraries(mrs_action_server action_manager_lib)

install(TARGETS 
  mrs_action_server
  DESTINATION lib/${PROJECT_NAME}
)

### Python support ###
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mrs_action/action_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

### Tests ###
if(ENABLE_TESTS)
  #find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(${PROJECT_NAME}_test test/unit/action_test.cpp)
  #target_link_libraries(${PROJECT_NAME}_test action_manager_lib)
  install(PROGRAMS
    test/mock/fake_goto_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/basic.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/short_flight.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/delay_report.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/multiple_goals.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/fail.py TIMEOUT 300)
endif()

ament_package()
