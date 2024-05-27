import os

from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, Shutdown

from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_dir = get_package_share_directory('tdlas_mapping')
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='tdlas_mapping',
            executable='generate_map',
            name='generate_map',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"rayMarchResolution": 2.0},
                {"lambda": 0.3},
                {"prior": 10.0},
                {"filepath": "/home/pepe/colcon_ws/tdlas/toy.json"},
                {"sensor_name": "sensorTF"},
                {"reflector_name": "reflectorTF"},
            ],
            on_exit=Shutdown()
        ),
    ])