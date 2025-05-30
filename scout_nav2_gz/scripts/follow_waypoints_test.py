#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import time
from resource_monitor import ResourceMonitor 
import csv
from datetime import datetime
import os
from rtf_monitor import RTFMonitor
import subprocess
import argparse


waypoints = yaml.safe_load('''
waypoints:
  - position: {x: 2.0, y: 1.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: -1.0}
  - position: {x: -1.0, y: -2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.57}
  - position: {x: 0.5, y: 3.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.57}
  - position: {x: -1.0, y: 7.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 2.50}
  - position: {x: -4.0, y: 8.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 3.14}
''')



def log_results(robot, sim_env, total_time, avg_cpu, avg_gpu, avg_rtf, csv_file):
    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_path = os.path.join(script_dir, csv_file)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Determine test_id by counting existing rows
    if os.path.exists(log_path):
        with open(log_path, "r") as f:
            existing_lines = sum(1 for line in f) - 1  # minus header
        test_id = existing_lines + 1
    else:
        test_id = 1

    # Write to CSV
    file_exists = os.path.isfile(log_path)
    with open(log_path, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow([
                "timestamp",
                "robot",
                "sim_env",
                "test_id",
                "navigation_time_sec",
                "avg_cpu_usage",
                "avg_gpu_usage",
                "avg_rtf"
            ])
        
        writer.writerow([
            timestamp,
            robot,
            sim_env,
            test_id,
            round(total_time, 2),
            round(avg_cpu, 2),
            round(avg_gpu, 2),
            round(avg_rtf, 4)

        ])

def spawn_object(name, x=0.0, y=0.0, z=0.0, sim_env="gazebo_fortress"):
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.4 0.4 0.5</size></box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>0.4 0.4 0.5</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

    if sim_env == "gazebo_fortress":
        # Escape quotes and remove newlines for Fortress inline SDF
        sdf_inline = sdf.replace('"', '\\"').replace('\n', '')
        req_arg = (
            f'sdf: "{sdf_inline}", '
            f'name: "{name}", '
            f'pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}, '
            f'allow_renaming: true'
        )
        cmd = [
            "ign", "service",
            "-s", "/world/empty/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req", req_arg
        ]
    else:  # gazebo_classic

        # Escape quotes and flatten for command line
        sdf_inline = sdf.replace('"', '\\"').replace('\n', '')

        # Prepare the full service call
        cmd = [
            "ros2", "service", "call", "/spawn_entity", "gazebo_msgs/SpawnEntity",
            f"{{name: \"{name}\", xml: \"{sdf_inline}\", robot_namespace: \"\", "
            f"initial_pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, reference_frame: \"world\"}}"
        ]

    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(f"[SPAWN SUCCESS] Object '{name}' spawned at ({x}, {y}, {z}) in {sim_env}")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"[SPAWN ERROR] Failed to spawn object '{name}' in {sim_env}:")
        print(e.stderr)


def main():
    
    parser = argparse.ArgumentParser(description="Waypoint navigation performance tester")
    parser.add_argument('--robot', type=str, default='scout', help='Robot name (e.g., novamob or scout)')
    parser.add_argument('--sim_env', type=str, default='gazebo_fortress', help='Simulation environment (gazebo_fortress or gazebo_classic)')
    parser.add_argument('--csv_file', type=str, default='navigation_results.csv', help='CSV file name to log results')
    parser.add_argument('--dynamic_obstacles', type=str, choices=['true', 'false'], default='false',
                        help='Enable dynamic obstacle spawning logic (true/false)')

    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()  

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    goal_poses = list(map(create_pose, waypoints["waypoints"]))


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    
    # Start monitoring CPU/GPU usage
    monitor = ResourceMonitor()
    monitor.start()
    start_time = time.time()


    # Start monitoring Gazebo RTF (Classic or Fortress)
    rtf_monitor = RTFMonitor(sim_env=args.sim_env.replace("gazebo_", ""))
    rtf_monitor.start()


    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    spawned_1 = False   # flag to prevent multiple spawns
    spawned_2 = False  
    spawned_3 = False  

    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and feedback.current_waypoint == 1 and not spawned_1 and args.dynamic_obstacles == 'true':
            spawn_object("obstacle_cube_1", x=0.5, y=-1.0, z=0.25, sim_env=args.sim_env)
            spawned_1 = True
        if feedback and feedback.current_waypoint == 2 and not spawned_2 and args.dynamic_obstacles == 'true':
            spawn_object("obstacle_cube_2", x=0.0, y=1.5, z=0.25, sim_env=args.sim_env)
            spawned_2 = True
        if feedback and feedback.current_waypoint == 3 and not spawned_3 and args.dynamic_obstacles == 'true':
            spawn_object("obstacle_cube_3", x=0.5, y=5.0, z=0.25, sim_env=args.sim_env)
            spawned_3 = True
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        end_time = time.time()
        total_duration = end_time - start_time
        #Print the total time
        print(f'Total navigation time: {total_duration:.2f} seconds')
        
        #Print the perfomance results
        monitor.stop()
        avg_cpu, avg_gpu = monitor.report()
        print(f"[RESOURCE MONITOR] Avg CPU Usage: {avg_cpu:.2f}%")
        print(f"[RESOURCE MONITOR] Avg GPU Usage: {avg_gpu:.2f}%")

        rtf_monitor.stop()
        avg_rtf = rtf_monitor.report()
        print(f"Average RTF: {avg_rtf:.4f}")


        # Log the results
        log_results(
            robot=args.robot,
            sim_env=args.sim_env,
            total_time=total_duration,
            avg_cpu=avg_cpu,
            avg_gpu=avg_gpu,
            avg_rtf=avg_rtf,
            csv_file=args.csv_file
)

    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)




if __name__ == '__main__':
    main()