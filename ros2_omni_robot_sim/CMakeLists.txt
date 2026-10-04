cmake_minimum_required(VERSION 3.8)
project(ros2_omni_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(test_cpp_publisher src/test_publisher.cpp)
ament_target_dependencies(test_cpp_publisher rclcpp std_msgs)
install(TARGETS test_cpp_publisher DESTINATION lib/${PROJECT_NAME})

set(KINEMATICS_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(kinematics src/kinematics.cpp)
ament_target_dependencies(kinematics ${KINEMATICS_DEPENDENCIES})
install(TARGETS kinematics DESTINATION lib/${PROJECT_NAME})

# install selected python code
# install(PROGRAMS scripts/subscriber.py
#         DESTINATION lib/${PROJECT_NAME})

# install all python code inside scripts folder and add exec permission
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                         GROUP_EXECUTE GROUP_READ
                         WORLD_EXECUTE WORLD_READ
        FILES_MATCHING PATTERN "*.py")

install(DIRECTORY
  launch
  urdf
  rviz
  worlds
  meshes
  config
  map
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
