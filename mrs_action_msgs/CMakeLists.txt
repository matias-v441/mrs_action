cmake_minimum_required(VERSION 3.8)
project(mrs_action_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)

set(action_files
  "action/Goto.action"
  "action/Path.action"
  "action/ReferenceStamped.action"
  DEPENDENCIES std_msgs mrs_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES ${dependencies}
 )

ament_package()
