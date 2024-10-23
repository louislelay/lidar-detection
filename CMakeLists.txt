cmake_minimum_required(VERSION 3.8)
project(lidar_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Test Node
add_executable(test_node src/node_template/node.cpp)
ament_target_dependencies(test_node rclcpp sensor_msgs)

# Preprocessing Node
add_executable(preprocessor_node 
  src/pcl_preprocessor/node.cpp
  src/pcl_preprocessor/filter.cpp)
ament_target_dependencies(preprocessor_node rclcpp sensor_msgs pcl_conversions pcl_ros)
target_include_directories(preprocessor_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_preprocessor)

# Ground Removal Node
add_executable(ground_removal_node 
  src/pcl_ground_removal/node.cpp
  src/pcl_ground_removal/ground_removal.cpp
  src/pcl_preprocessor/filter.cpp)
ament_target_dependencies(ground_removal_node rclcpp sensor_msgs pcl_conversions pcl_ros)
target_include_directories(ground_removal_node PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_preprocessor 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_ground_removal)

# Cluster Extraction Node
add_executable(cluster_extraction_node 
  src/pcl_cluster_extraction/node.cpp
  src/pcl_cluster_extraction/cluster_extraction.cpp
  src/pcl_ground_removal/ground_removal.cpp
  src/pcl_preprocessor/filter.cpp)
ament_target_dependencies(cluster_extraction_node rclcpp sensor_msgs pcl_conversions pcl_ros)
target_include_directories(cluster_extraction_node PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_cluster_extraction 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_preprocessor 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_ground_removal)

# Bounding Box Node
add_executable(bounding_box_node 
  src/pcl_bounding_box/node.cpp
  src/pcl_bounding_box/bounding_box.cpp
  src/pcl_cluster_extraction/cluster_extraction.cpp
  src/pcl_ground_removal/ground_removal.cpp
  src/pcl_preprocessor/filter.cpp)
ament_target_dependencies(bounding_box_node rclcpp sensor_msgs pcl_conversions pcl_ros visualization_msgs)
target_include_directories(bounding_box_node PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_bounding_box 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_cluster_extraction 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_preprocessor 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_ground_removal)

# Make node executable
install(TARGETS
  test_node
  preprocessor_node
  ground_removal_node
  cluster_extraction_node
  bounding_box_node
  DESTINATION lib/${PROJECT_NAME})

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
