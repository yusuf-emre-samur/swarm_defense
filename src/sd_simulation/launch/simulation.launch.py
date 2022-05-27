import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch


def flight_controller_node(i):
    return Node(
        package="sd_flight_controller",
        executable="sd_flight_controller",
        name=f"sd_flight_controller_{i}",
        parameters=[
            {"id": f"sd_drone_{i}"},
            {"use_sim_time": True}
        ],
        arguments=[

        ],
        namespace=f"sd_drone_{i}"
    )


def generate_launch_description():
    # package share
    pkg_share = FindPackageShare("sd_simulation")

    # launch args

    # gazebo world file in worlds folder
    world_launch_arg = DeclareLaunchArgument(
        "world",
        default_value="world.world",
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
            # PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")]),
            "world": LaunchConfiguration("world"),
            "verbose": "true",
            "pause": "true"
        }.items()
    )
    # create launch desc.
    ld = LaunchDescription()
    ld.add_action(world_launch_arg)
    ld.add_action(gzserver)

    # add flight controller for each drone
    num_drones = 1
    for i in range(1, num_drones+1):
        ld.add_action(flight_controller_node(i))

    return ld
