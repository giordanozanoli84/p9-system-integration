#!/usr/bin/env python

import rospy
from scipy.spatial import KDTree
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.cb_waypoints)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.cb_traffic)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.cb_obstacle)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.wp_index_closest_light = -1

        # using this function to have control of publishing rate
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # check if closest point is ahead or behind
        closest = np.array(self.waypoints_2d[closest_idx])
        prev = np.array(self.waypoints_2d[closest_idx - 1])
        car = np.array([x, y])

        direction = np.dot(closest - prev, car - closest)

        # if direction > 0 then the two vectors point the same direction
        # meaning the car is ahead the closest waypoint, if the direction is
        # less than zero, then the car is between the prev and closest waypoint
        # if dot product is zero then the closest is lying on the hyperplane
        # othogonal to the waypoints difference
        if direction > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)  # circular buffer

        return closest_idx

    def cb_pose(self, msg):
        self.pose = msg

    def cb_waypoints(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def cb_traffic(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.wp_index_closest_light = msg.data

    def cb_obstacle(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def generate_lane(self):

        lane = Lane()
        wp_idx_closest = self.get_closest_waypoint_idx()
        wp_idx_farthest = wp_idx_closest + LOOKAHEAD_WPS
        wps = self.base_waypoints.waypoints[wp_idx_closest: wp_idx_farthest]

        if self.wp_index_closest_light == -1 \
                or self.wp_index_closest_light >= wp_idx_farthest:
            # using slicing like a circular buffer
            lane.waypoints = self.base_waypoints.waypoints[wp_idx_closest: wp_idx_farthest]
        else:
            lane.waypoints = self.decelerate_wps(wps, wp_idx_closest)
        return lane

    def decelerate_wps(self, waypoints, wp_idx_closest):
        ret = list()
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            wp_idx_stop = max(self.wp_index_closest_light - wp_idx_closest - 2, 0)
            d = self.distance(waypoints, i, wp_idx_stop)
            v = math.sqrt(2 * MAX_DECEL * d)
            v = 0.0 if v < 1.0 else v

            p.twist.twist.linear.x = min(v, wp.twist.twist.linear.x)

            ret.append(p)

        return ret

    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
