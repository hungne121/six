#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf
from math import sqrt


class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer', anonymous=True)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.map = None
        self.laser_data = None
        self.robot_position = None

    def map_callback(self, msg):
        self.map = msg

    def laser_callback(self, msg):
        self.laser_data = msg

    def get_robot_position(self):
        listener = tf.TransformListener()
        try:
            (trans, _) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
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

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cell = data[index(x, y)]
                if cell == 0:  # Free space
                    neighbors = [
                        (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                        (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)
                    ]
                    for nx, ny in neighbors:
                        if data[index(nx, ny)] == -1:  # Unknown
                            wx = origin.position.x + (x + 0.5) * resolution
                            wy = origin.position.y + (y + 0.5) * resolution
                            frontiers.append((wx, wy))
                            break
        return frontiers

    def explore_frontiers(self):
        if self.map is None:
            rospy.loginfo("Waiting for map...")
            return

        frontiers = self.detect_frontiers()
        if not frontiers:
            rospy.loginfo("No frontiers found.")
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position.")
            return

        closest = min(frontiers, key=lambda f: sqrt((robot_pos[0] - f[0]) ** 2 + (robot_pos[1] - f[1]) ** 2))

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = closest[0]
        goal.pose.position.y = closest[1]
        goal.pose.orientation.w = 1.0

        rospy.loginfo(f"Publishing goal to /move_base_simple/goal: ({closest[0]:.2f}, {closest[1]:.2f})")
        self.goal_pub.publish(goal)

    def run(self):
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            self.explore_frontiers()
            rate.sleep()


if __name__ == '__main__':
    explorer = FrontierExplorer()
    explorer.run()
