cmake_minimum_required(VERSION 3.14)
project(gawron_plane_meta)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(gawron_filtering_msgs REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(GAWRON_PLANE_META_LIB_SRC
  src/gawron_plane_meta.cpp
)

set(GAWRON_PLANE_META_LIB_HEADERS
  include/gawron_plane_meta/gawron_plane_meta.hpp
  include/gawron_plane_meta/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GAWRON_PLANE_META_LIB_SRC}
  ${GAWRON_PLANE_META_LIB_HEADERS}
)

ament_export_include_directories(include)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(GAWRON_PLANE_META_NODE_SRC
  src/gawron_plane_meta_node.cpp
)

set(GAWRON_PLANE_META_NODE_HEADERS
  include/gawron_plane_meta/gawron_plane_meta_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GAWRON_PLANE_META_NODE_SRC}
  ${GAWRON_PLANE_META_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "perception::gawron_plane_meta::GawronPlaneMetaNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

target_link_libraries(gawron_plane_meta
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_gawron_plane_meta
    test/test_gawron_plane_meta.cpp
  )
  ament_target_dependencies(test_gawron_plane_meta ${TEST_SOURCES})

ament_target_dependencies(test_gawron_plane_meta ${TEST_SOURCES}
  rclcpp
  std_msgs 
  sensor_msgs
  pcl_conversions)

  target_link_libraries(test_gawron_plane_meta ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
