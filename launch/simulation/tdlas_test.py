"""
    Launch file to run GADEN gas dispersion simulator.
    IMPORTANT: GADEN_preprocessing should be called before!

    Parameters:
        @param scenario - The scenario where dispersal takes place
        @param simulation - The wind flow actuating in the scenario
        @param source_(xyz) - The 3D position of the release point
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.frontend.parse_substitution import parse_substitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    
    logger = LaunchConfiguration("log_level")    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Declare Arguments 
        # ==================
        DeclareLaunchArgument(
            "log_level", default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument(
            "namespace", default_value=["PioneerP3DX"],  #debug, info
            description="",
            ),
        #=======
        # NODES
        #=======
        Node(
            package='simulated_tdlas',
            executable='simulated_tdlas',
            name='simulated_tdlas',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'measurementFrequency' : 10.0,
                'reflectorLocTopic': '/Robot2/ground_truth'
                }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tdlas_tf_pub',
            output='screen',
            arguments = ['0', '0', '0.5', '-0.7', '0.7', '0', '0', parse_substitution('$(var namespace)_base_link'), 'tdlas_frame'],
            parameters=[{'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('test_env'),
                    'launch',
                    'gaden_player_launch.py'
                ])
            ),
            launch_arguments={
                'use_rviz': 'False',
            }.items()
        )
    ]) #end LaunchDescription
