#!/usr/bin/env python
import rospy
import math
import copy
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
import time

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        _ = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        _ = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        _ = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        _ = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.stop_line_positions = self.config['stop_line_positions']

        #self.upcuming_red_light_publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
	self.has_image = False
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.work_with_tl_classifier = False
        if self.work_with_tl_classifier:
            self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
            self.light_classifier = TLClassifier()
        else:
            self.upcuming_red_light_publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if self.pose and self.waypoints:
            rospy.loginfo("Current pos %s, %s", self.pose.pose.position.x, self.pose.pose.position.y)
            if self.lights and not self.work_with_tl_classifier:
                next_red_light_pose, next_red_light_state = self.get_next_red_light_pose()
                if next_red_light_pose:
                    rospy.loginfo("Next light pos %s, %s", next_red_light_pose.pose.pose.position.x, next_red_light_pose.pose.pose.position.y)
                    xy = self.get_closest_stop_line_position(next_red_light_pose)
                    rospy.loginfo("Stop line pos %s, %s", xy[0], xy[1])
                    index = self.get_closest_waypoint_xy(xy)
                    rospy.loginfo("publish = %s", index)
                    if next_red_light_state == TrafficLight.RED or next_red_light_state == TrafficLight.YELLOW:
                        self.upcuming_red_light_publisher.publish(Int32(index))
                    else:
                        self.upcuming_red_light_publisher.publish(Int32(-1))

    def pose_cb(self, msg):
        self.pose = msg
        quaternion = (self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = copy.deepcopy(waypoints.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def get_closest_stop_line_position(self, light_pose):
        """
        Find closest stop line position to the given position as a list of x, y coordinates. If no stop line position is provided return 0, 0
        """
        assert len(self.stop_line_positions) > 0
        curr_min_distance = 1.0E10
        curr_closest_pos = [0,0]
        for pos in self.stop_line_positions:
            x, y = pos[0] - light_pose.pose.pose.position.x, pos[1] - light_pose.pose.pose.position.y
            curr_dist = math.sqrt(x*x + y*y)
            if curr_dist < curr_min_distance:
                curr_min_distance = curr_dist
                curr_closest_pos = pos
        return curr_closest_pos

    def get_next_red_light_pose(self):
        """
        Find position of next traffic light
        """
        #red_lights = [l for l in self.lights if l.state == TrafficLight.RED]
        rospy.loginfo("get_next_red_light_pose")
        if (self.has_image):
            time0 = time.time()
            next_traffic_light_pose, next_traffic_light_state = self.get_light_state(None)
            time1 = time.time()
            rospy.loginfo("cost_time: %s ms, traffic light status: %s", 1000*(time1-time0), state)
            return next_traffic_light_pose, next_traffic_light_state
        else:
            return None, self.state

#        index = self.get_waypoint_ahead(self.pose.pose, self.lights)
#        index = index%len(self.lights)
#        next_traffic_light_pose = self.lights[index]
#        next_traffic_light_state = self.lights[index].state
#        return next_traffic_light_pose, next_traffic_light_state
        

    def get_waypoint_ahead(self, pose, waypoints):
        closest_dist = 100000
        closest_idx = 0
        for idx, wp in enumerate(waypoints):
            dist = self.distance(pose.position, wp.pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx
        p1 = pose.position
        p2 = waypoints[closest_idx].pose.pose.position
        heading = math.atan2((p2.y - p1.y), (p2.x - p1.x))
        angle = abs(self.current_yaw - heading)
        if angle > math.pi/4:
            closest_idx += 1
        return closest_idx

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
	rospy.loginfo("get_image_message")
        self.has_image = True
        self.camera_image = msg
#	time0 = time.time()
#        light_wp, state = self.process_traffic_lights()
#        time1 = time.time()
#	rospy.loginfo("cost_time: %s ms, traffic light status: %s", 1000*(time1-time0), state)
#
#        '''
#        Publish upcoming red lights at camera frequency.
#        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
#        of times till we start using it. Otherwise the previous stable state is
#        used.
#        '''
#        if self.state != state:
#            self.state_count = 0
#            self.state = state
#        elif self.state_count >= STATE_COUNT_THRESHOLD:
#            self.last_state = self.state
#            light_wp = light_wp if state == TrafficLight.RED else -1
#            self.last_wp = light_wp
#            self.upcoming_red_light_pub.publish(Int32(light_wp))
#        else:
#            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
#        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        closest_dist = 100000
        closest_idx = -1
        for idx, wp in enumerate(self.waypoints):
            dist = self.distance(pose.position, wp.pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx
        return closest_idx
    
    def get_closest_waypoint_xy(self, xy):
        closest_dist = 100000
        closest_idx = -1
        assert len(self.waypoints) > 0
        for idx, wp in enumerate(self.waypoints):
            x = xy[0] - wp.pose.pose.position.x
            y = xy[1] - wp.pose.pose.position.y
            dist = math.sqrt(x*x + y*y)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #Get classification
        return self.light_classifier.get_classification(cv_image)
        '''
        for tl in self.lights:
            if (distance(tl.pose.position, light.pose.position) < 0.1):
                return tl.state
        return TrafficLight.UNKNOWN
        '''

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #find the closest visible traffic light (if one exists)
        dist, state = self.get_light_state(light)
        if (dist == -1):
            return -1, TrafficLight.UNKNOWN
        else:
            stop_line = self.get_closest_stop_line_position(next_red_light_pose)
            rospy.loginfo("Stop line pos %s, %s", stop_line[0], stop_line[1])
            light_wp = self.get_closest_waypoint_xy(stop_line)
            rospy.loginfo("publish = %s", light_wp)
            return light_wp, state
        '''
        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN
        '''

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

