import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # package share
    pkg_share = FindPackageShare("simulation")

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
            #             "init": "false",
            #          "factory": "false",
            #            "force_system": "false",
            "verbose": "true"
        }.items()
    )
    return LaunchDescription([
        world_launch_arg,
        gzserver
    ])
