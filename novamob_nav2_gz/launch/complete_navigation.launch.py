import launch
import os
import launch_ros
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


# Lanch the robot and the navigation stack


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="novamob_nav2_gz"
    ).find("novamob_nav2_gz")
    # Messages are from: https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#launching-nav2
    diff_drive_loaded_message = "Successfully loaded controller diff_drive_base_controller into state active"
    toolbox_ready_message = "Registering sensor"
    navigation_ready_message = "Creating bond timer"
    default_model_path = os.path.join(pkg_share, "src/description/novamob_description.urdf")


    run_headless = LaunchConfiguration("run_headless")
    world = LaunchConfiguration("world")  
    use_trailer = LaunchConfiguration("use_trailer")
    model = LaunchConfiguration("model")


    # Including launch files with execute process
    bringup = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("novamob_nav2_gz"),
                    "launch",
                    "display.launch.py",
                ]
            ),
            "use_rviz:=false",
            ["run_headless:=", run_headless],
            ["world:=", world],
            "use_localization:=True",
            ["use_trailer:=", use_trailer],
            ["model:=", model],

        ],
        shell=False,
        output="screen",
    )

    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            ),
        ],
        shell=False,
        output="screen",
    )

    waiting_toolbox = RegisterEventHandler(
        OnProcessIO(
            target_action=bringup,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(
                        msg="Diff drive controller loaded. Starting SLAM Toolbox..."
                    ),
                    toolbox,
                ],
            ),
        )
    )

    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["params_file:=", LaunchConfiguration('params_file')]
        ],
        shell=False,
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(NotSubstitution(run_headless)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    waiting_navigation = RegisterEventHandler(
        OnProcessIO(
            target_action=toolbox,
            on_stdout=on_matching_output(
                # diff_drive_loaded_message,
                toolbox_ready_message,
                [
                    LogInfo(msg="SLAM Toolbox loaded. Starting navigation..."),
                    # TODO Debug: Navigation fails to start if it's launched right after the slam_toolbox
                    TimerAction(
                        period=20.0,
                        actions=[navigation],
                    ),
                    rviz_node,
                ],
            ),
        )
    )

    waiting_success = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [
                    LogInfo(msg="Ready for navigation!"),
                ],
            ),
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=[FindPackageShare("novamob_nav2_gz"), "/config/nav2_params_novamob.yaml"],
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("novamob_nav2_gz"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value=[
                    FindPackageShare("novamob_nav2_gz"),
                    "/world/ign_indoor/ign_indoor.sdf",
                ],
                description="Absolute path to the world file",
            ),
            DeclareLaunchArgument(
                name="use_trailer", 
                default_value="False",
                description="Use the robot model with a trailer",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            bringup,
            waiting_toolbox,
            waiting_navigation,
            waiting_success,
        ]
    )
