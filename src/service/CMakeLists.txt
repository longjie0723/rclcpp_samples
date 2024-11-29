cmake_minimum_required(VERSION 3.8)
project(service)

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
add_library(service_lib SHARED
  src/simple_lifecycle_node.cpp
  src/simple_node.cpp
)

# Add dependencies to the library
ament_target_dependencies(service_lib
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

# Register nodes for rclcpp_components
rclcpp_components_register_node(service_lib
  PLUGIN "SimpleLifecycle"  # Registering SimpleLifecycle plugin
  EXECUTABLE simple_lifecycle_node_exec
)

rclcpp_components_register_node(service_lib
  PLUGIN "SimpleNode"        # Registering SimpleNode plugin
  EXECUTABLE simple_node_exec
)

# Add executables
# add_executable(simple_lifecycle_node_exec src/simple_lifecycle_node.cpp)
# add_executable(simple_node_exec src/simple_node.cpp)

# Link the shared library to the executables
target_link_libraries(simple_lifecycle_node_exec service_lib)
target_link_libraries(simple_node_exec service_lib)

# Add ament dependencies for the executables
ament_target_dependencies(simple_lifecycle_node_exec
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

ament_target_dependencies(simple_node_exec
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)



# Install targets
install(TARGETS
  service_lib
  simple_lifecycle_node_exec
  simple_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # DESTINATION lib/${PROJECT_NAME}
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
