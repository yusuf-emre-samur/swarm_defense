import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch


def generate_launch_description():
    # create launch desc.
    ld = LaunchDescription()

    ###############################################################
    # Launch arguments
    ###############################################################
    # gazebo world file in worlds folder
    world_launch_arg = DeclareLaunchArgument(
        "world",
        default_value="world.world",
        description="Gazebo world file name in worlds folder of package."
    )
    ld.add_action(world_launch_arg)

    ###############################################################
    # gazebo server
    ###############################################################
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            ]),
        ]),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
            "pause": "true"
        }.items()
    )
    ld.add_action(gzserver)


    return ld
