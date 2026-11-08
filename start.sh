bash

cd /home/l/ros_ws
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
终端 2：启动 point_lio、backend 与 Nav2

bash

cd /home/l/ros_ws
source install/setup.bash
./stop_all.sh nav                      # 清掉上次的 component_container_isolated 等残留
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 \
  slam:=False \
  use_zloc:=True




ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2026 \
slam:=False

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2026 use_relocalization:=False slam:=False

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py


ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True


/red_standard_robot1

ros2 lifecycle set /red_standard_robot1/small_gicp_relocalization shutdown

方式一：持续重定位（默认行为，与原来一致）
small_gicp_relocalization:
  ros__parameters:
    enable_relocalization: true

方式二：只进行一次匹配
small_gicp_relocalization:
  ros__parameters:
    enable_relocalization: false



echo "" >> ~/.bashrc && echo "# Auto source ROS2 workspace setup" >> ~/.bashrc && echo "source install/setup.bash" >> ~/.bashrc

新版控制
编译包：
cd /home/l/ros_ws
colcon build --packages-select chassis_cmd_world --symlink-install

source 环境：
source install/setup.bash

运行程序：
ros2 run chassis_cmd_world test_chassis_cmd_world --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.0 -p w:=1.0


