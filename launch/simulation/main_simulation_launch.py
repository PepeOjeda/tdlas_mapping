# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    my_dir = get_package_share_directory('tdlas_mapping')
    
    rviz =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(my_dir, 'launch', 'tdlas_mapping.rviz')],
        output="log",
        prefix='xterm -hold -e',
        remappings=[
            ("/initialpose", "/giraff/initialpose"),
            ("/goal_pose", "/giraff/goal_pose")
        ]
    )

    coppelia_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('coppelia_ros2_pkg'),
                'launch/coppeliaSim.launch')
        ),
        launch_arguments={
            'coppelia_scene_path': PathJoinSubstitution([
                my_dir,
                'maps',
                LaunchConfiguration('scenario'),
                'coppeliaScene.ttt'
            ]),
            'coppelia_headless': 'True',
            'autoplay': 'True',
        }.items()
    )
    
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tdlas_mapping'),'launch', 'single_robot_nav.py' )
        ),
        launch_arguments={
            'namespace': 'Robot1',
            'scenario': LaunchConfiguration('scenario'),
            'nav_params_yaml': LaunchConfiguration('nav_params_yaml'),
        }.items()
    )
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tdlas_mapping'),'launch', 'single_robot_nav.py' )
        ),
        launch_arguments={
            'namespace': 'Robot2',
            'scenario': LaunchConfiguration('scenario'),
            'nav_params_yaml': LaunchConfiguration('nav_params_yaml'),
        }.items()
    )

    tdlas_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tdlas_mapping'),'launch', 'tdlas_test.py' )
        ),
        launch_arguments={
            'namespace': 'Robot1',
        }.items()
    )

    return[
        rviz,
        coppelia_launch,
        robot1,
        robot2,
        #tdlas_launch,
    ]

def generate_launch_description():

    my_dir = get_package_share_directory('tdlas_mapping')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument('scenario', default_value="Exp_C"), #required
        DeclareLaunchArgument('nav_params_yaml', default_value=os.path.join(my_dir, 'launch', 'nav2_params.yaml') ),
        OpaqueFunction(function = launch_setup)
    ])
