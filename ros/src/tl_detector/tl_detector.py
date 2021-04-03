#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import random
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.has_image = False
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.cb_waypoints)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.cb_traffic)
        rospy.Subscriber('/image_color', Image, self.cb_image)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.state_mapper = {
            TrafficLight.UNKNOWN: 'UNKNOWN',
            TrafficLight.RED: 'RED',
            TrafficLight.YELLOW: 'YELLOW',
            TrafficLight.GREEN: 'GREEN',
        }

        rospy.spin()

    def cb_pose(self, msg):
        self.pose = msg

    def cb_waypoints(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def cb_traffic(self, msg):
        self.lights = msg.lights

    def cb_image(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.logwarn('Caught traffic light WP at index {} with state {}'.format(
            light_wp, self.state_mapper[state]))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x : position to match a waypoint to in the x direction
            y : position to match a waypoint to in the y direction

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.waypoints_tree.query([x, y], 1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return random.choice([TrafficLight.RED, TrafficLight.YELLOW,
                              TrafficLight.GREEN])
        # return light.state
        # if not self.has_image:
        #     self.prev_light_loc = None
        #     return False
        #
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #
        # # get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.waypoints_tree:
            # cant do any logic about traffic lights positions
            return -1, TrafficLight.UNKNOWN

        closest_light = None
        wp_index_line = None

        # list of positions that correspond to the line to stop in front of
        # for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            wp_index_car = self.get_closest_waypoint(
                self.pose.pose.position.x, self.pose.pose.position.y)

            lookahead = len(self.waypoints.waypoints)
            for i, (light, line) in enumerate(zip(self.lights, stop_line_positions)):
                # since there are just a few traffic lights wps doesnt make any
                # sense to use a kdtree here, just iterate and find a closest wp
                # in the the neighborhood. also, the problem of getting a wp
                # behind the car here is less important as there are much less
                # wps and more spaced apart
                wp_index_tmp = self.get_closest_waypoint(line[0], line[1])
                diff = wp_index_tmp - wp_index_car
                if 0 <= diff < lookahead:
                    lookahead = diff
                    closest_light = light
                    wp_index_line = wp_index_tmp

        if closest_light:
            state = self.get_light_state(closest_light)
            return wp_index_line, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
