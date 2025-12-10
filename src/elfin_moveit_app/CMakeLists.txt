cmake_minimum_required(VERSION 3.8)
project(elfin_moveit_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(elfin_moveit_app src/elfin_moveit_app.cpp)
target_compile_features(elfin_moveit_app PUBLIC cxx_std_17)

ament_target_dependencies(elfin_moveit_app
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS elfin_moveit_app
  DESTINATION lib/${PROJECT_NAME})

# ← NUEVO: instala los archivos de launch si existen
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
