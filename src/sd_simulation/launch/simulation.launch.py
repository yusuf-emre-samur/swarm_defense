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

    ###############################################################
    # drones
    ###############################################################
    num_drones = 8
    for i in range(1, num_drones+1):

        # sd drone controller
        ld.add_action(
            Node(
                package="sd_drone_controller",
                executable="sd_drone_controller",
                name="sd_drone_controller",
                parameters=[
                    {"use_sim_time": True},
                    {"drone_id": i}
                ],
                arguments=[

                ],
                remappings=[
                    ("__ns", f"/drones/drone_{i}"),
                    ("__node", f"drone_{i}")
                ]
            )
        )
        # sd flight controller
        ld.add_action(
            Node(
                package="sd_flight_controller",
                executable="sd_flight_controller",
                name="sd_flight_controller",
                parameters=[
                    {"use_sim_time": True},
                    {"drone_id": i}
                ],
                arguments=[

                ],
                remappings=[
                    ("__ns", f"/drones/drone_{i}"),
                    ("__node", f"drone_{i}")
                ]
            )
        )
    return ld
