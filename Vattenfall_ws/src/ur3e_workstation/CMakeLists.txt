cmake_minimum_required(VERSION 3.8)
project(ur3e_workstation)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDFs and STL meshes
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/publish_collision_matrix.py
  scripts/follow_path_cartesian.py
  scripts/publish_environment.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
