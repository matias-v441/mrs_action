#  ------------------------------------------------------------------
#   Copyright 2024 Karelics Oy
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#  ------------------------------------------------------------------
import os
# ROS
import os.path
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, IfElseSubstitution, PathJoinSubstitution, EnvironmentVariable, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    custom_config = LaunchConfiguration('custom_config')  

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value=os.path.join(get_package_share_path("mrs_action"), "params", "mrs_action_defaults.yaml"),
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    ld.add_action(LogInfo(msg=['Config file: ', custom_config]))

    node = Node(
        name="action_manager",
        package="mrs_action",
        namespace=uav_name,
        #executable="action_manager_node.py",
        executable="mrs_action_server",
        emulate_tty=True,
        output={"both": {"screen", "log", "own_log"}},
        parameters=[custom_config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld.add_action(node)
    return ld

