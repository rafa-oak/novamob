import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
    PythonExpression,

)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="scout_nav2_gz"
    ).find("scout_nav2_gz")


    # EKF parameters path
    rl_params_file = os.path.join(
        pkg_share, "config", "dual_ekf_navsat_params.yaml")



    # Localize using odometry and IMU data. 
    # It can be turned off because the navigation stack uses AMCL with lidar data for localization
    robot_localization_node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": True}
        ],
        remappings=[
            ("odometry/filtered", "odometry/local")
        ],
    )

    robot_localization_node_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": True}
        ],
        remappings=[
            ("odometry/filtered", "odometry/local")
        ],
    )


    robot_localization_node_navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": True}
        ],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )


    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            robot_localization_node_odom,
            robot_localization_node_map,
            robot_localization_node_navsat,
        ]
    )
