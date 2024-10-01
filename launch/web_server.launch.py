#!/usr/bin/env python3
#
# Copyright 2024 KAIA.AI
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

import os, re
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def make_web_server_node(context: LaunchContext):
    package_name = 'kaiaai_web'
    web_server_config_path = os.path.join(
        get_package_share_path(package_name),
        'config',
        'web_server.yaml')
    print("Web server config : {}".format(web_server_config_path))

    return [
        Node(
            package=package_name,
            executable='web_server',
            name='web_server',
            parameters = [web_server_config_path],
            output='screen',
        )
    ]

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=make_web_server_node, args=[
        ])
    ])
