#!/usr/bin/env python3

import random
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Path
import time
import tf
from math import pi, atan2, sqrt

class RandomExplorer:
    def __init__(self):
        rospy.init_node("random_explorer")

        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.local_plan_callback)

        self.limit_x_p = 9 
        self.limit_x_n = -5.5
        self.limit_y_p = 7.3
        self.limit_y_n = -6.5
        self.resolution = 0.05
        self.exploration_interval = 5

        self.current_goal = None
        self.local_target = None
        self.listener = tf.TransformListener()

    def map_callback(self, data):
        self.map = data

    def local_plan_callback(self, msg):
        if msg.poses:
            last_pose = msg.poses[-1].pose
            self.local_target = (last_pose.position.x, last_pose.position.y)
        else:
            self.local_target = None


    def send_goal(self, goal):
        if goal is None:
            rospy.logwarn("No valid goal to send!")
            return

        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = "map"
        move_goal.target_pose.header.stamp = rospy.Time.now()
        move_goal.target_pose.pose.position.x = goal[0]
        move_goal.target_pose.pose.position.y = goal[1]
        move_goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: ({goal[0]}, {goal[1]})")

        self.move_base_client.send_goal(move_goal)
        self.current_goal = goal

    def generate_random_goal(self):
        if self.map is None:
            rospy.logwarn("Map is not yet available.")
            return None

        goal_x = random.uniform(self.limit_x_n, self.limit_x_p)
        goal_y = random.uniform(self.limit_y_n, self.limit_y_p)

        rospy.loginfo(f"Generated random goal: ({goal_x:.2f}, {goal_y:.2f})")
        return (goal_x, goal_y)

    def check_goal_validity(self, goal):
        if self.map is None:
            rospy.logwarn("Map data is not available.")
            return False

        width = self.map.info.width
        height = self.map.info.height
        x_grid = int((goal[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        y_grid = int((goal[1] - self.map.info.origin.position.y) / self.map.info.resolution)

        if 0 <= x_grid < width and 0 <= y_grid < height:
            idx = y_grid * width + x_grid
            if self.map.data[idx] == -1:  # unexplored
                return True
        rospy.logwarn("Goal is invalid or occupied.")
        return False

    def find_obstacle_on_path(self, goal):
        if self.map and self.local_target:
            x_grid = int((goal[0] - self.map.info.origin.position.x) / self.map.info.resolution)
            y_grid = int((goal[1] - self.map.info.origin.position.y) / self.map.info.resolution)

            idx = y_grid * self.map.info.width + x_grid
            if self.map.data[idx] >= 100:  # unexplored
                return True
            
        return False


    def trajectory_checking(self):
        """Monitor the robot on the way to the goal."""
        # Constantly checking on the status of current goal on the way ( 2-3 sec )
        # If goal => 0/100 => break
        # If find walls on the global path ? => break 
        # find new goal 

        check_interval = 2.0  # seconds
        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()

            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("Goal aborted or rejected.")
                
                return False

            if not self.check_goal_validity(self.current_goal):
                rospy.logwarn("Goal area is now invalid. Cancelling current goal...")
                self.move_base_client.cancel_goal()
                return False
            
            if self.find_obstacle_on_path(self.local_target):
                rospy.logwarn("Local goal blocked! Cancelling current goal...")
                self.move_base_client.cancel_goal()
                return False

            time.sleep(check_interval)

    def run(self):
        rospy.loginfo("Random Explorer started!")

        while not rospy.is_shutdown():
            rospy.loginfo("Generating a new random goal...")

            goal = self.generate_random_goal()

            if not goal:
                rospy.logwarn("Generated goal is None, trying again.")
                continue

            if self.check_goal_validity(goal):
                self.send_goal(goal)
                rospy.loginfo("Goal sent, now monitoring trajectory...")
                success = self.trajectory_checking()

                if not success:
                    rospy.logwarn("Finding a new goal due to failure...")
                    continue  # try finding a new one immediately

                rospy.loginfo(f"Goal succeeded! Waiting for {self.exploration_interval} seconds before next goal.")
                time.sleep(self.exploration_interval)

            else:
                rospy.logwarn("Goal invalid, generating another goal...")


if _name_ == "_main_":
    explorer = RandomExplorer()
    explorer.run()
