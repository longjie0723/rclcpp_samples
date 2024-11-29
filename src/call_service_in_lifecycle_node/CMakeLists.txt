cmake_minimum_required(VERSION 3.8)
project(call_service_in_lifecycle_node)

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

# Add the library (if necessary)
add_library(call_service_in_lifecycle_node_lib SHARED
  src/simple_lifecycle.cpp
)

ament_target_dependencies(call_service_in_lifecycle_node_lib
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

# Add the executable
add_executable(call_service_in_lifecycle_node
  src/simple_lifecycle.cpp
)

# register node 
# rclcpp_components_register_node(call_service_in_lifecycle_node_lib
#   PLUGIN "SimpleLifecycle"
#   EXECUTABLE call_service_in_lifecycle_node
# )


# Link the library to the executable
target_link_libraries(call_service_in_lifecycle_node
  call_service_in_lifecycle_node_lib
)

# Add ament dependencies for the executable
ament_target_dependencies(call_service_in_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  example_interfaces
  rclcpp_components
  std_srvs
)

# Install targets
install(TARGETS
  call_service_in_lifecycle_node
  call_service_in_lifecycle_node_lib
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Enable testing if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (adjust if you add licenses)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (adjust if this package is in a git repository)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark this as an ament package
ament_package()
