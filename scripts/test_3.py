#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from nav_msgs.msg import OccupancyGrid
from random import uniform
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

        # Some internal state
        self.map = None
        self.robot_position = None

    def map_callback(self, msg):
        # Store the map
        self.map = msg

    def get_robot_position(self):
        # Get the current robot's position using tf
        listener = tf.TransformListener()
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Robot poisition failed")
            return None

    def check_goal_validity(self, x, y):
        """Check if the random goal is inside the map and unexplored"""
        width = self.map.info.width
        height = self.map.info.height
        # Convert (x, y) to map indices
        ix = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        iy = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

        # Ensure the point is within the bounds
        if 0 <= ix < width and 0 <= iy < height:
            index = iy * width + ix
            if self.map.data[index] == 0:  # 0 means unexplored
                return True
        return False

    def generate_random_goal(self):
        """Generate a random goal within the map's bounds"""
        x = uniform(-12, 12)
        y = uniform(-12, 12)

        # Check if the goal is valid
        if self.check_goal_validity(x, y):
            return x, y
        else:
            return None

    def send_goal(self, x, y):
        """Send a goal to the move_base"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal

        # Log the goal information
        rospy.loginfo(f"Sending goal to move_base with coordinates: x={x}, y={y}")
        rospy.loginfo(f"Goal Pose: {goal}")

        self.move_base_client.send_goal(move_goal)
        self.move_base_client.wait_for_result()

        result = self.move_base_client.get_result()
        if result:
            rospy.loginfo("Goal reached!")
        else:
            rospy.logwarn("Failed to reach the goal")

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.map:
                # Generate a random goal
                random_goal = self.generate_random_goal()

                if random_goal:
                    rospy.loginfo(f"Generated goal: {random_goal}")
                    self.send_goal(random_goal[0], random_goal[1])

                rate.sleep()


if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()

# goal can be near wall  in the scanned area
