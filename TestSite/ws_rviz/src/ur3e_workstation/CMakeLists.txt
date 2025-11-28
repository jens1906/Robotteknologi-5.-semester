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

# Install Python scripts
install(PROGRAMS
  scripts/publish_collision_matrix.py
  scripts/path_follower.py
  scripts/path_execution_service.py
  scripts/test_path_publisher.py
  scripts/testplate_path_publisher.py
  scripts/move_to_single_point.py
  scripts/follow_path_ompl.py
  scripts/follow_path_corridor.py
  scripts/follow_path_cartesian.py
  scripts/line_simulator.py
  scripts/line_visualizer.py
  scripts/follow_line_cartesian.py
  scripts/point_visualizer.py
  scripts/single_point_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
