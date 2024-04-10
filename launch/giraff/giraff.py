import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        

        Node(
            package='nav2_over_mqtt',
            executable='nav2MqttSender',
            name='nav2MqttSender',
            output='screen',
            namespace="giraff",
            parameters=[
                {"goalTopic":"/giraff/NavToPose"},
                {"resultTopic":"/giraff/NavigationResult"},
            ]  
        ),
    ])