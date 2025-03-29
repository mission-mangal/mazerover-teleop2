#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
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
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def launch_setup(context):
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)
    gz_world = LaunchConfiguration("gz_world").perform(context)

    gz_args = f"-r -v {gz_log_level} {gz_world}"
    if eval(gz_headless_mode):
        gz_args = "--headless-rendering -s " + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return [gz_sim]


def generate_launch_description():

    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")

    declare_gz_headless_mode = DeclareLaunchArgument(
        "gz_headless_mode",
        default_value="True",
        description="Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the amount of calculations.",
        choices=["True", "False"],
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    declare_gz_log_level = DeclareLaunchArgument(
        "gz_log_level",
        default_value="2",
        description="Adjust the level of console output.",
        choices=["0", "1", "2", "3", "4"],
    )

    declare_gz_world_arg = DeclareLaunchArgument(
        "gz_world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("leo_gz_worlds"), "worlds", "maze1.sdf"]
        ),
        description="Absolute path to SDF world file.",
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )


    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_gz_headless_mode,
            declare_gz_log_level,
            declare_gz_world_arg,
            topic_bridge,
            robot_ns,
            spawn_robot,
            OpaqueFunction(function=launch_setup),
        ]
    )
