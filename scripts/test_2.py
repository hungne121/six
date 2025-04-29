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
        rospy.init_node('frontier_explorer', anonymous=False)

        self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.map = None
        self.robot_position = None

    def map_callback(self, msg):
        self.map = msg

    def laser_callback(self, msg):
        self.laser_data = msg

    def get_robot_position(self):
        # Get the current robot's position using tf
        listener = tf.TransformListener()
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def detect_frontiers(self):
        frontiers = []

        if self.map is None:
           return frontiers

        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin = self.map.info.origin
        data = self.map.data

        def index(x, y):
           return x + y * width

        for y in range(1, height-1):
            for x in range(1, width-1):
                cell_value = data[index(x, y)]

                if cell_value == 0:  # Free space
                    # Check 8 neighbors
                    neighbors = [
                        (x+1, y), (x-1, y), (x, y+1), (x, y-1),
                        (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)
                    ]

                    for nx, ny in neighbors:
                        neighbor_value = data[index(nx, ny)]
                        if neighbor_value == -1:  # Unknown neighbor
                            # Found a frontier cell
                            wx = origin.position.x + (x + 0.5) * resolution
                            wy = origin.position.y + (y + 0.5) * resolution
                            frontiers.append((wx, wy))
                            break  # No need to check other neighbors

        return frontiers

    def explore_frontiers(self):
        if self.map is None:
           rospy.loginfo("Waiting for map...")
           return

        frontiers = self.detect_frontiers()
        if not frontiers:
            rospy.loginfo("No frontiers found!")
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position.")
            return

        closest_frontier = min(frontiers, key=lambda f: sqrt((robot_pos[0] - f[0])**2 + (robot_pos[1] - f[1])**2))

        frontier_goal = PoseStamped()
        frontier_goal.header.frame_id = "map"
        frontier_goal.pose.position.x = closest_frontier[0]
        frontier_goal.pose.position.y = closest_frontier[1]
        frontier_goal.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending frontier goal: ({closest_frontier[0]:.2f}, {closest_frontier[1]:.2f})")

        self.send_goal_to_movebase(frontier_goal)

    def is_goal_valid(self, pose):
        if self.map is None:
            rospy.logwarn("No map received yet, assuming goal is valid.")
            return True

        map_x = int((pose.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
        map_y = int((pose.position.y - self.map.info.origin.position.y) / self.map.info.resolution)

        index = map_y * self.map.info.width + map_x

        if index < 0 or index >= len(self.map.data):
            rospy.logwarn("Goal is out of map bounds!")
            return False

        cost = self.map.data[index]

        if cost == -1:
            rospy.logwarn("Goal is in an unknown area, not validated yet.")
            return True  # Still allow, because you're *exploring*

        if cost >= 50:
            rospy.logwarn("Goal is in an occupied space or too close to an obstacle!")
            return False

        return True

    def send_goal_to_movebase(self, goal):
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal

        rospy.loginfo("Sending goal to move_base")

        self.move_base_client.send_goal(move_goal)

        rate = rospy.Rate(2)  # 2 Hz check while moving
        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()

            if state == 3:  # Goal reached
               rospy.loginfo("Goal reached!")
               break
            elif state == 4 or state == 5:  # Aborted or rejected
               rospy.logwarn("Goal was aborted or rejected by move_base.")
               break

            if not self.is_goal_valid(goal.pose):
               rospy.logwarn("Goal became invalid while navigating! Cancelling...")
               self.move_base_client.cancel_goal()
               break

            rate.sleep()


    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.explore_frontiers()
            rate.sleep()

if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()
