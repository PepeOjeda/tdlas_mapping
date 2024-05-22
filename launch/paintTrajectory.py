import os

from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction

from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_dir = get_package_share_directory('tdlas_mapping')
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    
        Node(
            package='tdlas_mapping',
            executable='paint_trajectory',
            name='paint_trajectory',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"filepath": "/home/pepe/colcon_ws/tdlas/logs/tdlasLog_2_1.json"},
            ]
        ),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                #{'yaml_filename' : os.path.join(my_dir, "maps", "parking.yaml")},
                {'frame_id' : 'map'}
                ],
            ),
        # LIFECYCLE MANAGER
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            ]
                        }
            ]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', os.path.join(my_dir, 'launch', 'trajectory.rviz')],
            output="log",
            prefix='xterm -hold -e',
        ),
    ])