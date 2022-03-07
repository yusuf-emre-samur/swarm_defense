import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch

def generate_launch_description():
    # package share
    pkg_share = FindPackageShare("sd_simulation")

    # launch args

    # gazebo world file in worlds folder
    world_launch_arg = DeclareLaunchArgument(
        "world",
        default_value="world1.world",
        description="Gazebo world file name in worlds folder of package."
    )

    # start gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            ]),
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")]),
            "verbose": "true",
            "pause": "true"
        }.items()
    )

    sd_flight_controller = Node(
        package="sd_flight_controller",
        executable="flight_controller",
        name="sd_flight_controller_1",
        parameters=[
            {"drone_pose_topic": "/sd_drone_1/pose"}
        ]
    )



    return LaunchDescription([
        world_launch_arg,
        gzserver,
        sd_flight_controller
    ])
