cmake_minimum_required(VERSION 3.8)
project(parameterization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ParameterizationStatus.msg"
  "msg/UVPoint.msg"
  "msg/UVBoundary.msg"
  "srv/InterpolatePoint.srv"
  "srv/GetUVBounds.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

# Find Python3
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install Python dependencies from requirements.txt before building
# Using --break-system-packages is safe here because we're installing development dependencies
execute_process(
  COMMAND ${Python3_EXECUTABLE} -m pip install --break-system-packages -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
  RESULT_VARIABLE PIP_INSTALL_RESULT
  OUTPUT_QUIET
  ERROR_QUIET
)

if(PIP_INSTALL_RESULT EQUAL 0)
  message(STATUS "Successfully installed Python dependencies from requirements.txt")
else()
  message(WARNING "Could not install Python dependencies automatically. Please run: pip install -r requirements.txt")
endif()

# Get the python install dir
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

# Install Python source files to the package directory
install(FILES
  parameterization/__init__.py
  parameterization/surface_parameterization.py
  parameterization/parameterization_node.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Install Python executables
install(PROGRAMS
  parameterization/parameterization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME parameterization_node
)

# Install launch files (if they exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install examples (if they exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
  install(DIRECTORY examples
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
