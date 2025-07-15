# Copyright (c) 2023 OUXT Polaris
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
import pytest
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory

@pytest.mark.launch_test
def generate_test_description():
    robotx_bt_share_dir = get_package_share_directory('robotx_behavior_tree')
    config_filepath = os.path.join(robotx_bt_share_dir, 'config', 'go_around_object.yaml')
    navi_sim_share_dir = get_package_share_directory('navi_sim')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'navi_sim', 'with_planner.launch.py',
                 f'behavior_config_filepath:={config_filepath}'],
            cwd=navi_sim_share_dir,     
            output='screen',
            name='navi_sim_launch_process',
        ),
        ReadyToTest(),
    ])