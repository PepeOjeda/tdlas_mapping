import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    node = Node(
        package="tdlas_mapping",
        executable="log_measurements",
        #prefix ="",
        parameters=[
           {"file_path":"tdlasLog_4.json"},
           
           {"tdlasTopic":"/falcon/reading"},
           
           {"fixedFrame":"map"},
           {"sensorFrame":"camera"},
           {"reflectorFrame":"gps_hunter"},

           {"arucoTopic":"/methane/aruco/detections"},
           {"aimingThresholdPx": 300.0},
           {"targetPx_X": int(1920/2)},
           {"targetPx_Y": int(1080/2)},
        ],
    )
    return [
        node,
    ]


def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)