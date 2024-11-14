# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory(
        "scout_nav2_gz")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    default_world_path = os.path.join(gps_wpf_dir, "world/outdoor.sdf")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    world_path = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_trailer = LaunchConfiguration('use_trailer')
    log_level = LaunchConfiguration('log_level')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    
    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')
    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x', default_value='-20.0', description='X-coordinate for robot spawn position')
    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y', default_value='10.0', description='Y-coordinate for robot spawn position')
    declare_spawn_z_cmd = DeclareLaunchArgument(
        'spawn_z', default_value='1.0', description='Z-coordinate for robot spawn position')
    declare_world_path_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(launch_dir, default_world_path), description='World file path for Gazebo')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation time')
    declare_use_trailer_cmd = DeclareLaunchArgument(
        'use_trailer', default_value='False', description='Use trailer')    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn', description='Log level for ROS nodes')
    declare_gz_verbosity_cmd = DeclareLaunchArgument(
        'gz_verbosity', default_value='3', description='Gazebo verbosity level (0-4)')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo_gps_world.launch.py')),
        launch_arguments={
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'use_sim_time': use_sim_time,
            'use_trailer': use_trailer,
            'log_level': log_level,
            'gz_verbosity': gz_verbosity,
            'world': world_path
        }.items()
    )
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_y_cmd)
    ld.add_action(declare_spawn_z_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)    
    ld.add_action(declare_use_trailer_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_gz_verbosity_cmd)
    # simulator launch
    ld.add_action(gazebo_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)


    return ld
