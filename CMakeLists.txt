cmake_minimum_required(VERSION 3.8)
project(rb1_shelf_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rb1_shelf_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# servers
add_executable(find_shelf_service_server_node src/find_shelf_service_server.cpp)
ament_target_dependencies(find_shelf_service_server_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs nav_msgs rb1_shelf_msgs tf2 tf2_ros)  

add_executable(approach_shelf_service_server_node src/approach_shelf_service_server.cpp)
ament_target_dependencies(approach_shelf_service_server_node rclcpp rb1_shelf_msgs tf2_ros tf2_geometry_msgs tf2)

add_executable(init_localization_service_server_node src/init_localization_service_server.cpp)
ament_target_dependencies(init_localization_service_server_node rclcpp rb1_shelf_msgs tf2_ros tf2_geometry_msgs tf2)

add_executable(find_nav_points_service_server_node src/find_nav_points_service_server.cpp)
ament_target_dependencies(find_nav_points_service_server_node rclcpp nav_msgs rb1_shelf_msgs tf2_ros geometry_msgs tf2)

################
add_executable(bt_ros2_test src/bt_ros2_test.cpp)
ament_target_dependencies(bt_ros2_test rclcpp behaviortree_cpp geometry_msgs std_srvs rb1_shelf_msgs)


add_executable(bt_ros2_async src/bt_ros2_async.cpp)
ament_target_dependencies(bt_ros2_async rclcpp behaviortree_cpp geometry_msgs std_srvs visualization_msgs std_msgs nav_msgs rb1_shelf_msgs tf2 tf2_ros nav2_msgs tf2_geometry_msgs)

add_executable(bt_selector_behavior_tree src/bt_selector_behavior_tree.cpp)
ament_target_dependencies(bt_selector_behavior_tree rclcpp behaviortree_cpp visualization_msgs geometry_msgs std_srvs std_msgs nav_msgs rb1_shelf_msgs tf2 tf2_ros nav2_msgs tf2_geometry_msgs)
install(TARGETS
  
  #servers
  init_localization_service_server_node
  find_shelf_service_server_node
  approach_shelf_service_server_node
  find_nav_points_service_server_node
  ## bts
  bt_ros2_async
  bt_ros2_test
  bt_selector_behavior_tree

  DESTINATION lib/${PROJECT_NAME})


install(
    DIRECTORY 
    bt_xml 
    launch
    config
    DESTINATION share/${PROJECT_NAME})
  


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
