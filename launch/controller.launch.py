"""Launch one controller per Crazyflie."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _controller_nodes(context):
    total_robots = int(LaunchConfiguration("total_robots").perform(context))
    robot_prefix = LaunchConfiguration("robot_prefix").perform(context)
    scenario_radius = float(LaunchConfiguration("scenario_radius").perform(context))
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
                    "constraint_mode": constraint_mode,
                }
            ],
        )
        for robot_id in range(total_robots)
    ]


def generate_launch_description():
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
                "constraint_mode",
                default_value="full",
                description="Constraint mode: full or vo_only.",
            ),
            TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=_controller_nodes)],
            ),
        ]
    )
