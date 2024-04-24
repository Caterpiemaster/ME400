# Copyright 2020 ros2_control Development Team
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
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF via xacro
    use_gazebo_parameter_name = 'use_gazebo'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    
    use_gazebo = LaunchConfiguration(use_gazebo_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)

    capstone_xacro_file = os.path.join(get_package_share_directory('diffdrive_arduino'), 'urdf',
                                     'capstone.urdf.xacro')
    
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', capstone_xacro_file, ' use_gazebo:=', use_gazebo,
         ' use_fake_hardware:=', use_fake_hardware])


    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_arduino"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    robot_controllers]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value="false" ,
            description="use gazebo simulation"),
        ##############################################
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value="false",
            description="use fake hardware interface. default is true for safety"),
        robot_state_pub_node,
        # joystick,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
