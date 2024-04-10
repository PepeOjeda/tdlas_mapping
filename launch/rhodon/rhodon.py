import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    my_dir = get_package_share_directory('tdlas_mapping')
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='nav2_over_mqtt',
            executable='nav2MqttSender',
            name='nav2MqttSender',
            output='screen',
            namespace="rhodon",
            parameters=[
                {"goalTopic":"/rhodon/NavToPose"},
                {"resultTopic":"/rhodon/NavigationResult"},
            ]  
        ),

        Node(
            package="tdlas_mapping",
            executable="test_node",
            name="test_node",
            output="screen",
            parameters=[
                {"followerDistance":1.5},
            ]  
        ),
    ])