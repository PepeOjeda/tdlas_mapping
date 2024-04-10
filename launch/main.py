import os

from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction

from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_dir = get_package_share_directory('tdlas_mapping')
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_dir, 'launch', 'rhodon', 'rhodon.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_dir, 'launch', 'giraff', 'giraff.py')
            )
        ),
        
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
            #prefix='xterm -hold -e',
            parameters=[
                {"host":"150.214.109.137"},
                {"port":8002},
                {"MQTT_namespace":"pc"},
                {"MQTT_topics_subscribe":"/rhodon/NavigationResult,/giraff/NavigationResult"},
            ]            
        ),

        # MAP
        ##############
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', os.path.join(my_dir, 'launch', 'simulation', 'tdlas_mapping.rviz')],
            output="log",
            #prefix='xterm -hold -e',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename' : os.path.join(my_dir, "maps", "Mapirlab", "occupancy.yaml")},
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

    ])