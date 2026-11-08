cmake_minimum_required(VERSION 3.8)
project(zloc_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/map_odom_fallback.py
    scripts/cloud_self_filter.py
    scripts/fake_lio_publisher.py
    scripts/grab_scan.py
    scripts/inspect_map_bundle.py
    scripts/zloc_pose_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
