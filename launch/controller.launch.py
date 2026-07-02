"""Launch one placeholder controller per Crazyflie."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _controller_nodes(context):
    total_robots = int(LaunchConfiguration("total_robots").perform(context))
    robot_prefix = LaunchConfiguration("robot_prefix").perform(context)
    scenario_radius = float(LaunchConfiguration("scenario_radius").perform(context))
    preferred_velocity = float(LaunchConfiguration("preferred_velocity").perform(context))
    constraint_mode = LaunchConfiguration("constraint_mode").perform(context)

    return [
        Node(
            package="pmr_tp_final",
            executable="controller",
            name=f"cbf_agent_controller_{robot_id}",
            output="screen",
            parameters=[
                {
                    "robot_id": robot_id,
                    "robot_prefix": robot_prefix,
                    "total_robots": total_robots,
                    "scenario_radius": scenario_radius,
                    "preferred_velocity": preferred_velocity,
                    "constraint_mode": constraint_mode,
                }
            ],
        )
        for robot_id in range(total_robots)
    ]


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("pmr_tp_final"),
        "rviz",
        "simul.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "total_robots",
                default_value="4",
                description="Number of Crazyflie controller nodes to start.",
            ),
            DeclareLaunchArgument(
                "robot_prefix",
                default_value="cf_",
                description="Prefix used to build robot names, e.g. cf_0.",
            ),
            DeclareLaunchArgument(
                "scenario_radius",
                default_value="1.0",
                description="Radius of the opposite-circle goal scenario.",
            ),
            DeclareLaunchArgument(
                "preferred_velocity",
                default_value="0.2",
                description="Preferred speed for the straight-line reference.",
            ),
            DeclareLaunchArgument(
                "constraint_mode",
                default_value="full",
                description="Constraint mode: full or vo_only.",
            ),
            TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=_controller_nodes)],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
