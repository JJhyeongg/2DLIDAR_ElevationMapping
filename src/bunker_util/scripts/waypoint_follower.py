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

import csv
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
"""
Navigation demo to follow a given path after smoothing
"""

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    # Generate goal poses
    goal_poses = []
    y_values = [0, 2, 5, 8, 10, 12, 15, 18, 20, 22, 25, 28, 30, 32]
    for i, y in enumerate(y_values):
        if i % 2 == 0:  # 짝수 인덱스: 순방향, w=1
            for x in range(0, 33, 2):
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(x)
                goal_pose.pose.position.y = float(y)
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = 0.0 
                goal_pose.pose.orientation.w = 1.0
                goal_poses.append(goal_pose)
        else:  # 홀수 인덱스: 역방향, y=1
            for x in range(32, -1, -2):
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(x)
                goal_pose.pose.position.y = float(y)
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = 1.0  
                goal_pose.pose.orientation.w = 0.0
                goal_poses.append(goal_pose)

    print(f"Total number of waypoints: {len(goal_poses)}")

    # Save goal poses to a CSV file
    with open('goal_poses.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])  # Header
        for pose in goal_poses:
            writer.writerow([pose.pose.position.x, pose.pose.position.y])

    # Navigate to each goal pose
    for i, goal_pose in enumerate(goal_poses):
        print(f"Navigating to goal {i+1}/{len(goal_poses)}")
        
        # Get the path and smooth it
        path = navigator.getPath(initial_pose, goal_pose)
        smoothed_path = navigator.smoothPath(path)

        # Follow the smoothed path
        navigator.followPath(smoothed_path)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    'Estimated distance remaining to goal position: '
                    + '{0:.3f}'.format(feedback.distance_to_goal)
                    + '\nCurrent speed of the robot: '
                    + '{0:.3f}'.format(feedback.speed)
                )

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')
            break

        # Update initial_pose for the next iteration
        initial_pose = goal_pose

    # navigator.lifecycleShutdown()
    # exit(0)

if __name__ == '__main__':
    main()