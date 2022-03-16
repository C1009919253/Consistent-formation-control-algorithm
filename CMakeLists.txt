cmake_minimum_required(VERSION 3.5)
project(three_aircraft_control)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hpipm REQUIRED)
find_package(blasfeo REQUIRED)
#find_package(Eigen REQUIRED)
#find_package(cpaad REQUIRED)

include_directories(include)

set(msg_files
  "msg/Offsets.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(three_aircraft_control_demo
  src/three_aircraft_control.cpp
  src/Main_Controler.cpp
)

ament_target_dependencies(three_aircraft_control_demo
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "ct_core"
  "ct_rbd"
  "ct_optcon"
  "ct_models"
)

install(TARGETS three_aircraft_control_demo
 DESTINATION lib/${PROJECT_NAME})
 
rosidl_target_interfaces(three_aircraft_control_demo
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
