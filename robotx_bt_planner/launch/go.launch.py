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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    navi_sim_dir = os.path.join(
        get_package_share_directory('navi_sim'), 'launch')
    description = LaunchDescription([
        # launch navi sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navi_sim_dir, '/with_planner.launch.py']),
        ),
        Node(
            package='robotx_bt_planner',
            executable='robotx_bt_planner_node',
            name='robotx_bt_planner_node',
            parameters=[{'config_package':'robotx_bt_planner',
                         'config_file':'config/go.yaml',
                         'update_rate':30.0}]
        )
    ])
    return description

