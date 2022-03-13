cmake_minimum_required(VERSION 3.16.3)
project(bt_ros_example)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    behaviortree_cpp_v3
    std_msgs
    rclcpp
    rclcpp_components
)

include_directories(  
  include
  ${PCL_INCLUDE_DIRS} 
)

add_library(test_bt SHARED 
  src/subscriber_input.cpp
  src/subscriber_input_coro.cpp
)
target_include_directories(test_bt
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_bt ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(test_bt)

add_library(ros_node SHARED src/ros_node.cpp)
target_include_directories(ros_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ros_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_node
                      test_bt
)

add_executable(ros_node_main src/ros_node_main.cpp)
target_include_directories(ros_node_main
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ros_node_main ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_node_main
                      ros_node
)

# Install libraries
install(
  TARGETS
    test_bt
    ros_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS
    ros_node_main
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
