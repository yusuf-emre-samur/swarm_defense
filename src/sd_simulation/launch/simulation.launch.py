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
    base_station_positions = [
        [-100, -100, 0.175],
        [-100, -102, 0.175],
        [-100, -104, 0.175],
        [-100, -106, 0.175],
        [-102, -100, 0.175],
        [-102, -102, 0.175],
        [-102, -104, 0.175],
        [-102, -106, 0.175],

    ]
    min_fly = 3
    battery = [100.0 for i in range(min_fly)] + \
        [10.0 * i for i in range(num_drones-min_fly)]


    for i in range(num_drones):

        # sd drone controller
        ld.add_action(
            Node(
                package="sd_drone_controller",
                executable="sd_drone_controller",
                name="sd_drone_controller",
                parameters=[
                    {"use_sim_time": True},
                    {"drone_id": i},
                    {"base_station_pos": base_station_positions[i]},
                    {"min_flying_drones": min_fly},
                    {"battery": battery[i]}
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
        # sd  communication
        ld.add_action(
            Node(
                package="sd_communication",
                executable="sd_communication",
                name="sd_communication",
                parameters=[
                    {"use_sim_time": True},
                    {"drone_id": i},
                    {"old_after": 10}
                ],
                arguments=[

                ],
                remappings=[
                    ("__ns", f"/drones/drone_{i}"),
                    ("__node", f"drone_{i}")
                ]
            )
        )
    ld.add_action(
        Node(
            package="sd_visualiser",
            executable="sd_visualiser",
            name="sd_visualiser",
            parameters=[
                    {"num_drones": num_drones}
            ],
            arguments=[

            ],
            remappings=[
                ("__ns", f"/drones/visualiser"),
                ("__node", f"visualiser")
            ]
        )
    )
    return ld
