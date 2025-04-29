#!/usr/bin/env python3

import random
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from nav_msgs.msg import OccupancyGrid
import time
import tf
from math import pi, atan2, sqrt

class RandomExplorer:
    def __init__(self):
        rospy.init_node("random_explorer")

        # Initialize the SimpleActionClient for move_base
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Subscribe to the costmap and map
        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Boundaries for goal generation
        self.limit_x_p = 9 
        self.limit_x_n = -5.5
        self.limit_y_p = 7.3
        self.limit_y_n = -6.5
        self.resolution = 0.05  # Example resolution in meters (adjust as needed)
        self.exploration_interval = 5  # Interval to check for obstacles and find a new goal        

    def map_callback(self, data):
        self.map = data

    def send_goal(self, goal):
        """Send a goal to the robot and log progress"""
        if goal is None:
            rospy.logwarn("No valid goal to send!")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0] 
        goal_msg.pose.position.y = goal[1] 
        goal_msg.pose.orientation.w = 1.0

        rospy.loginfo(f"Attempting to send goal: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal_msg

        self.move_base_client.send_goal(move_goal)
        rospy.loginfo("Goal sent, waiting for result...")

        self.move_base_client.wait_for_result()
        result = self.move_base_client.get_result()

        if result:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach the goal.")

    def generate_random_goal(self):
        """Generate a random goal within the map boundaries and log the result"""
        if self.map is None:
            rospy.logwarn("Map is not yet available.")
            return None

        # Generate a random goal within the specified bounds
        goal_x = random.uniform(self.limit_x_n, self.limit_x_p)
        goal_y = random.uniform(self.limit_y_n, self.limit_y_p)

        rospy.loginfo(f"Generated random goal: ({goal_x}, {goal_y})")
        return (goal_x, goal_y)

    def check_goal_validity(self, goal):
        """Check if the goal is valid (unexplored and not too close to walls)"""
        if self.map is None:
            rospy.logwarn("Map data is not available.")
            return False

        width = self.map.info.width
        height = self.map.info.height
        # Convert (x, y) to map indices
        x_grid = int((goal[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        y_grid = int((goal[1] - self.map.info.origin.position.y) / self.map.info.resolution)

        # Ensure the point is within the bounds
        if 0 <= x_grid < width and 0 <= y_grid < height:
            idx = y_grid * width + x_grid
            if self.map.data[idx] == -1:  # -1 means unexplored
                return True
        rospy.logwarn("Goal is invalid or outside map bounds.")
        return False

    def run(self):
        """Run the explorer"""
        rospy.loginfo("Random Explorer started!")

        while not rospy.is_shutdown():
            rospy.loginfo("Generating a new random goal...")

            goal = self.generate_random_goal()

            if not goal:
                continue

            if self.check_goal_validity(goal):
                rospy.loginfo(f"Sending goal: {goal}")
                self.send_goal(goal)
                
            else:
                rospy.logwarn("Goal invalid, trying again...")

            rospy.loginfo(f"Waiting for {self.exploration_interval} seconds before generating next goal.")
            time.sleep(self.exploration_interval)

if __name__ == "__main__":
    explorer = RandomExplorer()
    explorer.run()
