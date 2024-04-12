cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
)

add_library(map_manager SHARED
  src/map_manager.cpp
)

ament_target_dependencies(map_manager
  ${dependencies}
)

target_compile_definitions(map_manager PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS map_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_manager_server src/map_manager_server.cpp)
target_link_libraries(map_manager_server map_manager)
ament_target_dependencies(map_manager_server ${dependencies})

install(TARGETS
  map_manager_server
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
                  config 
                  maps 
        DESTINATION share/${PROJECT_NAME})

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
