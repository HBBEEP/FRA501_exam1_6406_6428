cmake_minimum_required(VERSION 3.8)
project(fibo_turtlesim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(PROGRAMS
  scripts/controller.py
  scripts/scheduler.py
  scripts/fibo_via_point_gen.py
  scripts/turtle_standby.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  # add directories here
  launch 
  DESTINATION share/${PROJECT_NAME})

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGoal.srv"
  DEPENDENCIES geometry_msgs 
)

ament_package()
