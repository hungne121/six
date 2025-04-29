#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from nav_msgs.msg import OccupancyGrid  # Add this import
from sensor_msgs.msg import LaserScan
import tf
from math import pi, atan2, sqrt



class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer', anonymous=True)

        # Create a SimpleActionClient to move the robot using move_base
        self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Some internal state
        self.map = None
        self.robot_position = None

    def map_callback(self, msg):
        # Store the map
        self.map = msg

    def laser_callback(self, msg):
        # Optionally use laser scan data for obstacle detection
        self.laser_data = msg

    def get_robot_position(self):
        # Get the current robot's position using tf
        listener = tf.TransformListener()
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def explore_frontiers(self):
        if self.map is None:
            rospy.loginfo("Waiting for map...")
            return

        # Here you would implement the frontier detection algorithm
        # For now, we'll just go to a random location for exploration
        frontier_goal = PoseStamped()
        frontier_goal.header.frame_id = "map"
        frontier_goal.pose.position.x = -5.0  # Just an example
        frontier_goal.pose.position.y = -5.0  # Just an example
        frontier_goal.pose.orientation.w = 1.0

        # Use SimpleActionClient to send the goal to move_base
        self.send_goal_to_movebase(frontier_goal)

    def send_goal_to_movebase(self, goal):
        # Create a MoveBaseGoal message
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal

        rospy.loginfo("Sending goal to move_base")

        # Send goal and wait for result
        self.move_base_client.send_goal(move_goal)
        self.move_base_client.wait_for_result()

        # Get the result of the movement
        result = self.move_base_client.get_result()

        if result:
            rospy.loginfo("Goal reached!")
        else:
            rospy.logwarn("Failed to reach the goal")

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.explore_frontiers()
            rate.sleep()

if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()

# this code runs successfully, but there're 2 problems
# 1. It's not self-exploring: the frontier is chosen first => frontier detection + frontier exploration 
# (In frontier_explorer.py, but now still has error calculating frontier => the frontier goal is in bad location)
# 2. Sometimes the local planner can fall into the never-ending rotate recovery process 