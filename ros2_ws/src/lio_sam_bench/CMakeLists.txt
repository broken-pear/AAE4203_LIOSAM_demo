cmake_minimum_required(VERSION 3.8)
project(lio_sam_bench)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gt_sync.py
  scripts/traj_recorder.py
  scripts/evaluate.py
  scripts/make_figures.py
  scripts/topic_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
