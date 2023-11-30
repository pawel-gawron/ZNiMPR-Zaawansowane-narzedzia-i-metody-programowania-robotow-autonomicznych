cmake_minimum_required(VERSION 3.14)
project(gawron_node_service)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

find_package(gawron_service REQUIRED)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(GAWRON_NODE_SERVICE_LIB_SRC
  src/gawron_node_service.cpp
)

set(GAWRON_NODE_SERVICE_LIB_HEADERS
  include/gawron_node_service/gawron_node_service.hpp
  include/gawron_node_service/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GAWRON_NODE_SERVICE_LIB_SRC}
  ${GAWRON_NODE_SERVICE_LIB_HEADERS}
)

set(GAWRON_NODE_SERVICE_NODE_SRC
  src/gawron_node_service_node.cpp
)

set(GAWRON_NODE_SERVICE_NODE_HEADERS
  include/gawron_node_service/gawron_node_service_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GAWRON_NODE_SERVICE_NODE_SRC}
  ${GAWRON_NODE_SERVICE_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "gawron_node_service::GawronNodeServiceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_gawron_node_service
    test/test_gawron_node_service.cpp
  )
  ament_target_dependencies(test_gawron_node_service ${TEST_SOURCES})
  target_link_libraries(test_gawron_node_service ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
