from launch import LaunchDescription 
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue


from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os 

def generate_launch_description():
    arm_description=get_package_share_directory("arm_description")
    arm_description_share=get_package_prefix("arm_description")
    gazebo_ros_dir=get_package_share_directory("gazebo_ros")
    
    model_path=os.path.join(arm_description,"models")+":"+os.path.join(arm_description_share,"share")
    env_var=SetEnvironmentVariable('GAZEBO_MODEL_PATH',model_path)
    
    model_arg=DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("arm_description"),"urdf","arm_urdf.xacro"),
        description="Absoulate path to the robot URDF Xacro File"
    )
    robot_description=ParameterValue(Command(["xacro ",LaunchConfiguration("model")]),value_type=str)
    
    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description" : robot_description}]
    )
    
    start_gazebo_server=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir,"launch","gzserver.launch.py")
        )
    )
    
    start_gazebo_client=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir,"launch","gzclient.launch.py")
        )
    )
    
    spawn_robot=Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","armbot",
            "-topic","robot_description",
            ],
        output="screen"
    )
    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_robot
    ])
