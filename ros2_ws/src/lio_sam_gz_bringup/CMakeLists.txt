cmake_minimum_required(VERSION 3.8)
project(lio_sam_gz_bringup)

find_package(ament_cmake REQUIRED)

# 场景 / 模型 / 参数 / RViz 配置 / launch 文件
install(DIRECTORY
  launch
  config
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Python 节点
install(PROGRAMS
  scripts/sensor_prep.py
  scripts/patrol_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
