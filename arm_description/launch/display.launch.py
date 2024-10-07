from launch import LaunchDescription #launch file return a LaunchDescription object in which all Nodes included
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node #For creating nodes 
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory #for finding the directory path
import os #Taking arguments and finding path

def generate_launch_description():
    
    model_arg=DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("arm_description"),"urdf","arm_urdf.xacro"),
        description="Absoulate path to the robot URDF Xacro File"
    )
    
    robot_description=ParameterValue(Command(["xacro ",LaunchConfiguration("model")])) #converting xacro into launchable format
    
    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )
    
    joint_state_publisher=Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz_node=Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("arm_description"),"rviz","display.rviz")]
        
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
        
    ])