from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare which model to use (burger, waffle, waffle_pi)

    my_robot_dir = get_package_share_directory("my_robot")
    # model = LaunchConfiguration("model")

    # Package paths
    # tb3_desc_dir = get_package_share_directory("turtlebot3_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")


    # Path to your custom URDF
    urdf_file = os.path.join(my_robot_dir, "urdf", "my_robot.urdf")

    # Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        )
    )

    # Robot State Publisher (publishes TF + robot_description)
    state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_file).read()}],
        output="screen"
    )

    # Spawn entity in Gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "my_robot",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        state_pub,
        spawn
    ])
