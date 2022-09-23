# Copyright (c) 2022 OUXT Polaris
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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os


def generate_launch_description():
    behavior_config_package = LaunchConfiguration(
        "behavior_config_package", default="robotx_bt_planner"
    )
    behavior_config_filepath = LaunchConfiguration(
        "behavior_config_filepath", default="config/go.yaml"
    )
    behavior_update_rate = LaunchConfiguration("behavior_update_rate", default=20.0)
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "behavior_config_package",
                default_value=behavior_config_package,
                description="ros package name which behavior config exists",
            ),
            DeclareLaunchArgument(
                "behavior_config_filepath",
                default_value=behavior_config_filepath,
                description="config yaml path for robot behavior",
            ),
            DeclareLaunchArgument(
                "behavior_update_rate",
                default_value=behavior_update_rate,
                description="update rate of robot behavior",
            ),
            ComposableNodeContainer(
                name="behavior_bringup_container",
                namespace="behavior",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="robotx_costmap_calculator",
                        plugin="robotx_costmap_calculator::CostmapFilterComponent",
                        namespace="perception",
                        name="costmap_filter_node",
                        parameters=[
                            {
                                "config_package": behavior_config_package,
                                "config_file": behavior_config_filepath,
                                "update_rate": behavior_update_rate,
                            }
                        ],
                    )
                ],
                output="screen",
            ),
        ]
    )
