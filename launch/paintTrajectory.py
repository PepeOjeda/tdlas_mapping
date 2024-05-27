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
                {"filepath": "/home/pepe/HDD/colcon_ws/tdlas/1_1.json"},
            ]
        ),


        Node(
            package='gps2cartesian',
            executable='fakeGPSpub',
            name='fakeGPSpub',
            output='screen',
            parameters=[
                {'gps_latitude' : 36.7159195233333},
                {'gps_longitude' : -4.47891364833333},
                {'gps_altitude' : 0.0},
                {'gps_frame_id' : 'map'},
                {'gps_topic_pub' : 'fake_gps'}
                ],
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