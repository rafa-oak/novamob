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



def log_results(robot, sim_env, total_time, avg_cpu, avg_gpu, avg_rtf):
    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_path = os.path.join(script_dir, "navigation_results.csv")

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



def main():
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


    # Start monitoring Gazebo RTF
    rtf_monitor = RTFMonitor(sim_env='fortress')
    rtf_monitor.start()


    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

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
            robot="novamob",
            sim_env="gazebo_fortress",
            total_time=total_duration,
            avg_cpu=avg_cpu,
            avg_gpu=avg_gpu,
            avg_rtf=avg_rtf
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