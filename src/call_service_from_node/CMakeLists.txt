cmake_minimum_required(VERSION 3.8)
project(call_service_from_node)

# Add compile options for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)

# Add the shared library
add_library(simple_node_lib SHARED
  src/simple_node.cpp
)

# Add dependencies to the library
ament_target_dependencies(simple_node_lib
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

# Register nodes for rclcpp_components
# rclcpp_components_register_node(call_service_from_lifecycle
#   PLUGIN "SimpleLifecycle"  # Registering SimpleLifecycle plugin
#   EXECUTABLE simple_node_lib
# )

# rclcpp_components_register_node(call_service_from_lifecycle
#   PLUGIN "SimpleNode"        # Registering SimpleNode plugin
#   EXECUTABLE simple_node_exec
# )

# Add executables
# add_executable(simple_node_lib src/simple_lifecycle_node.cpp)
add_executable(simple_node src/simple_node.cpp)

# Link the shared library to the executables
target_link_libraries(simple_node simple_node_lib)

# Add ament dependencies for the executables
ament_target_dependencies(simple_node_lib
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

# Install targets
install(TARGETS
  simple_node_lib
  simple_node
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files (uncomment if necessary)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Enable testing if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark this as an ament package
ament_package()
