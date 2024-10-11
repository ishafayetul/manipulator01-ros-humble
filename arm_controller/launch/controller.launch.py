from launch import LaunchDescription 
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue


from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os 

def generate_launch_description():
    
    robot_description=ParameterValue(
        Command([
            "xacro ", os.path.join(get_package_share_directory("arm_description"),"urdf","arm_urdf.xacro")
        ]),
        value_type=str
    )
    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )
    join_state_broadcaster_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    arm_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    gripper_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    return LaunchDescription([
        robot_state_publisher,
        join_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])