cmake_minimum_required(VERSION 3.14)
project(gawron_plane_seg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# add_compile_options(-Wall -Wextra -pedantic -Werror)

set(GAWRON_PLANE_SEG_LIB_SRC
  src/gawron_plane_seg.cpp
)

set(GAWRON_PLANE_SEG_LIB_HEADERS
  include/gawron_plane_seg/gawron_plane_seg.hpp
  include/gawron_plane_seg/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GAWRON_PLANE_SEG_LIB_SRC}
  ${GAWRON_PLANE_SEG_LIB_HEADERS}
)

set(GAWRON_PLANE_SEG_NODE_SRC
  src/gawron_plane_seg_node.cpp
)

set(GAWRON_PLANE_SEG_NODE_HEADERS
  include/gawron_plane_seg/gawron_plane_seg_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GAWRON_PLANE_SEG_NODE_SRC}
  ${GAWRON_PLANE_SEG_NODE_HEADERS}
)

ament_export_include_directories(include)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "perception::gawron_plane_seg::GawronPlaneSegNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

target_link_libraries(gawron_plane_seg
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_gawron_plane_seg
    test/test_gawron_plane_seg.cpp
  )
  ament_target_dependencies(test_gawron_plane_seg ${TEST_SOURCES})
  target_link_libraries(test_gawron_plane_seg ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
