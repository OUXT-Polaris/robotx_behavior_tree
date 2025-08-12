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
import unittest

import pytest
import rclpy
from std_msgs.msg import String

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory

# このLaunchDescriptionがテスト対象のシステム（シミュレーション環境）を起動します。 
# launch_testing は、この記述に従ってシステムを起動した後に、後述のテストクラスを実行します。
@pytest.mark.launch_test
def generate_test_description():
    robotx_bt_share_dir = get_package_share_directory('robotx_behavior_tree')
    config_filepath = os.path.join(robotx_bt_share_dir, 'config', 'go_around_object.yaml')
    navi_sim_share_dir = get_package_share_directory('navi_sim')

    # シミュレーション環境を起動するプロセスを定義
    navi_sim_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'navi_sim', 'with_planner.launch.py',
             f'behavior_config_filepath:={config_filepath}'],
        cwd=navi_sim_share_dir,
        output='screen',
        name='navi_sim_launch_process',
    )

    return LaunchDescription([
        navi_sim_process,
        # ReadyToTestアクションは、上記のプロセスが起動した後に
        # 後続のテストケース(GoAroundObjectIntegrationTestクラス)を実行するよう指示します。
        ReadyToTest(),
    ])

# こちらがテストケース本体です。
# unittest.TestCaseを継承し、ROS 2ノードとして振る舞いながらアサーションを行います。
class GoAroundObjectIntegrationTest(unittest.TestCase):

    def setUp(self):
        # テスト実行前にrclpyを初期化し、テスト用ノードを作成します。
        rclpy.init()
        self.node = rclpy.create_node('test_go_around_object_subscriber')
        self.msg_received = None
        # C++のテストと同様に、/go_around_object/started トピックを購読します。
        self.subscription = self.node.create_subscription(
            String,
            '/go_around_object/started',
            self.callback,
            10)

    def tearDown(self):
        # テスト実行後にノードを破棄し、rclpyをシャットダウンします。
        self.node.destroy_node()
        rclpy.shutdown()

    def callback(self, msg: String):
        # メッセージを受信したら、その内容をクラスのメンバー変数に保存します。
        self.node.get_logger().info(f'Received message: "{msg.data}"')
        self.msg_received = msg

    def test_receive_start_message(self):
        # メッセージを受信するまで最大80秒間待機します。
        timeout_sec = 80.0
        rclpy.spin_until_future_complete(
            self.node, rclpy.Future(), timeout_sec=timeout_sec)

        # 受信したメッセージを検証します。
        self.assertIsNotNone(self.msg_received, "Timeout: Did not receive a message!")
        self.assertEqual(self.msg_received.data, "GoAroundObject node started successfully!")