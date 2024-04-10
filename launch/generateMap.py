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
                {"rayMarchResolution": 0.4},
                {"lambda": 0.0},
                {"filepath": os.path.join(my_dir, "data", "measurement_log1")},
            ],
            on_exit=Shutdown()
        ),
    ])