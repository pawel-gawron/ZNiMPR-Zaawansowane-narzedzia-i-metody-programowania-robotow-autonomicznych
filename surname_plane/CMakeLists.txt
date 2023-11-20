cmake_minimum_required(VERSION 3.14)
project(surname_plane)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(SURNAME_PLANE_LIB_SRC
  src/surname_plane.cpp
)

set(SURNAME_PLANE_LIB_HEADERS
  include/surname_plane/surname_plane.hpp
  include/surname_plane/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SURNAME_PLANE_LIB_SRC}
  ${SURNAME_PLANE_LIB_HEADERS}
)

set(SURNAME_PLANE_NODE_SRC
  src/surname_plane_node.cpp
)

set(SURNAME_PLANE_NODE_HEADERS
  include/surname_plane/surname_plane_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SURNAME_PLANE_NODE_SRC}
  ${SURNAME_PLANE_NODE_HEADERS}
)
ament_export_include_directories(include)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "surname_plane::SurnamePlaneNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

target_link_libraries(surname_plane
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_surname_plane
    test/test_surname_plane.cpp
  )
  ament_target_dependencies(test_surname_plane ${TEST_SOURCES}
  rclcpp
  std_msgs 
  sensor_msgs
  pcl_conversions)
  
  target_link_libraries(test_surname_plane ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
