# Copyright 2023 Martin Nievas
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
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="start RViz automatically with the launch file",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("htb_description"), "urdf", "htb_gazebo.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    htb_diff_drive_controller = os.path.join(
        get_package_share_directory("htb_bringup"),
        "config",
        "htb_diff_drive_controller.yaml",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("htb_description"), "config", "htb.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory('htb_bringup'),
        'config',
        'gazebo_params.yaml'
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'htb'],
                        output='screen')

    delay_spawn_entity_after_gazebo_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[spawn_entity],
        )
    )


    return LaunchDescription(
        [
            arg_show_rviz,
            node_robot_state_publisher,
            rviz_node,
            gazebo,
            spawn_entity,
        ]
    )