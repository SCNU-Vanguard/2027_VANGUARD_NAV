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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    output_frame_id = LaunchConfiguration("output_frame_id")
    translation_x = LaunchConfiguration("translation_x")
    translation_y = LaunchConfiguration("translation_y")
    translation_z = LaunchConfiguration("translation_z")
    rotation_roll = LaunchConfiguration("rotation_roll")
    rotation_pitch = LaunchConfiguration("rotation_pitch")
    rotation_yaw = LaunchConfiguration("rotation_yaw")

    declare_input_topic_cmd = DeclareLaunchArgument(
        "input_topic",
        default_value="global_costmap/costmap",
        description="Input costmap topic",
    )

    declare_output_topic_cmd = DeclareLaunchArgument(
        "output_topic",
        default_value="global_costmap/costmap_transformed",
        description="Output transformed costmap topic",
    )

    declare_output_frame_id_cmd = DeclareLaunchArgument(
        "output_frame_id",
        default_value="map",
        description="Frame ID for the output costmap",
    )

    declare_translation_x_cmd = DeclareLaunchArgument(
        "translation_x", default_value="0.0", description="Translation in X axis (meters)"
    )

    declare_translation_y_cmd = DeclareLaunchArgument(
        "translation_y", default_value="0.0", description="Translation in Y axis (meters)"
    )

    declare_translation_z_cmd = DeclareLaunchArgument(
        "translation_z", default_value="0.0", description="Translation in Z axis (meters)"
    )

    declare_rotation_roll_cmd = DeclareLaunchArgument(
        "rotation_roll", default_value="0.0", description="Rotation around X axis (radians)"
    )

    declare_rotation_pitch_cmd = DeclareLaunchArgument(
        "rotation_pitch", default_value="0.0", description="Rotation around Y axis (radians)"
    )

    declare_rotation_yaw_cmd = DeclareLaunchArgument(
        "rotation_yaw", default_value="0.0", description="Rotation around Z axis (radians)"
    )

    costmap_transformer_node = Node(
        package="costmap_transformer",
        executable="costmap_transformer_node",
        name="costmap_transformer",
        output="screen",
        parameters=[
            {"input_topic": input_topic},
            {"output_topic": output_topic},
            {"output_frame_id": output_frame_id},
            {"translation_x": translation_x},
            {"translation_y": translation_y},
            {"translation_z": translation_z},
            {"rotation_roll": rotation_roll},
            {"rotation_pitch": rotation_pitch},
            {"rotation_yaw": rotation_yaw},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_input_topic_cmd)
    ld.add_action(declare_output_topic_cmd)
    ld.add_action(declare_output_frame_id_cmd)
    ld.add_action(declare_translation_x_cmd)
    ld.add_action(declare_translation_y_cmd)
    ld.add_action(declare_translation_z_cmd)
    ld.add_action(declare_rotation_roll_cmd)
    ld.add_action(declare_rotation_pitch_cmd)
    ld.add_action(declare_rotation_yaw_cmd)
    ld.add_action(costmap_transformer_node)

    return ld
