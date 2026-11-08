# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_relocalization = LaunchConfiguration("use_relocalization")
    use_zloc = LaunchConfiguration("use_zloc")
    zloc_config = LaunchConfiguration("zloc_config")
    zloc_prior_map = LaunchConfiguration("zloc_prior_map")

    # 条件组合用 Python 表达式，避免不同 launch 版本缺少 Not/And 替换类。
    zloc_on = PythonExpression(["('", use_zloc, "').lower() == 'true'"])
    # small_gicp 与静态 map→odom 只在「未启用 zloc」时按原逻辑生效。
    small_gicp_on = PythonExpression(
        [
            "not (('",
            use_zloc,
            "').lower() == 'true') and (('",
            use_relocalization,
            "').lower() == 'true')",
        ]
    )
    static_map_odom_on = PythonExpression(
        [
            "not (('",
            use_zloc,
            "').lower() == 'true') and not (('",
            use_relocalization,
            "').lower() == 'true')",
        ]
    )


    lifecycle_nodes = ["map_server"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_relocalization_cmd = DeclareLaunchArgument(
        "use_relocalization",
        default_value="True",
        description=(
            "Whether to run small_gicp_relocalization. If False, small_gicp is not launched and a "
            "static identity tf (map->odom) is published instead, so that the odom origin "
            "coincides with the map origin."
        ),
    )

    declare_use_zloc_cmd = DeclareLaunchArgument(
        "use_zloc",
        default_value="False",
        description=(
            "用 Z-Localization 后端替换 small_gicp 作为 map→odom 的唯一来源。"
            "启用后 small_gicp 与静态 map→odom 都不再启动，改由 backend 发布，"
            "并在其后端冻结时由 zloc_bridge 兜底。"
        ),
    )

    declare_zloc_config_cmd = DeclareLaunchArgument(
        "zloc_config",
        default_value=os.path.join(bringup_dir, "config", "simulation", "zloc_backend.yaml"),
        description="Z-Localization 后端配置文件路径",
    )

    declare_zloc_prior_map_cmd = DeclareLaunchArgument(
        "zloc_prior_map",
        default_value=os.path.join(bringup_dir, "pcd", "simulation", "rmuc_2026_bundle"),
        description="Z-Localization 先验地图包目录（build_pcd_map_bundle 的输出）",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # 里程计前端：point_lio（BSD-3，逐点更新）。
    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                condition=IfCondition(small_gicp_on),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    # NOTE: static_transform_publisher is NOT available as a composable node, so it has to be
    # launched as a standalone process in both composition and non-composition modes.
    # It is placed outside `load_nodes` so that it is started regardless of `use_composition`.
    start_static_map_to_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        condition=IfCondition(static_map_odom_on),
        # Must follow the same clock as the rest of the stack, otherwise the
        # static transform stamped with wall-clock time cannot be matched
        # against simulated-time lookups and `map` appears to not exist.
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # Z-Localization 后端：作为 map→odom 的唯一权威来源，替换 small_gicp。
    # 它通过 legacy 接口读取 loam_interface 输出的 lidar_odometry 与 registered_scan，
    # 因此 point_lio 与地图资产都不需要改。
    start_zloc_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("backend"), "launch", "backend.launch.py")
        ),
        condition=IfCondition(zloc_on),
        launch_arguments={
            "config_file": zloc_config,
            "use_sim_time": use_sim_time,
            "mode": "localization",
            "prior_map": zloc_prior_map,
            "rviz": "false",
        }.items(),
    )

    # 后端在 INITIALIZING / SUSPECT / LOST 期间不广播 map→odom，
    # 这里兜底保持 TF 链不断，否则 Nav2 会整栈停摆。
    start_map_odom_fallback = Node(
        package="zloc_bridge",
        executable="map_odom_fallback.py",
        name="map_odom_fallback",
        output="screen",
        condition=IfCondition(zloc_on),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "odom_frame": "odom",
                # 相对话题名：本节点与 backend 处在同一命名空间下。
                "state_topic": "backend_node/localization_state",
            }
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_relocalization_cmd)
    ld.add_action(declare_use_zloc_cmd)
    ld.add_action(declare_zloc_config_cmd)
    ld.add_action(declare_zloc_prior_map_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_map_to_odom_node)
    ld.add_action(start_zloc_backend)
    ld.add_action(start_map_odom_fallback)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
