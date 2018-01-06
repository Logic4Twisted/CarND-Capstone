#!/usr/bin/env python

import rospy
import math
import copy
import numpy as np
import tf
import yaml

from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight


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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.base_waypoints = None
        self.current_pose = None
        self.current_yaw = None
        self.max_speed = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        # By default, index value of -1 means traffic light is NOT red
        self.traffic_light_wp_idx = -1
        self.frame_id = None
        
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        quaternion = (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        
        new_lane = Lane()
        new_lane.header.frame_id = self.frame_id
        new_lane.header.stamp = rospy.Time(0)
        
        next_waypoint_idx = self.get_waypoint_ahead(self.current_pose, self.base_waypoints)
        end_waypoint_idx = min(len(self.base_waypoints)-1, next_waypoint_idx+LOOKAHEAD_WPS)
        new_waypoints = copy.deepcopy(self.base_waypoints[next_waypoint_idx:end_waypoint_idx])
        
        waypoints_to_light = self.traffic_light_wp_idx - 2 - next_waypoint_idx  # stop ahead with some buffer
        
        if len(new_waypoints) < LOOKAHEAD_WPS: # reaching end of track
            new_waypoints = self.decelerate(new_waypoints, len(new_waypoints)-1)
        elif self.traffic_light_wp_idx != -1 and waypoints_to_light < LOOKAHEAD_WPS:
            new_waypoints = self.decelerate(new_waypoints, waypoints_to_light)
        else:
            for i in range(len(new_waypoints)):
                self.set_waypoint_velocity(new_waypoints, i, self.max_speed)
        
        new_lane.waypoints = new_waypoints
        self.final_waypoints_pub.publish(new_lane)
        
    def waypoints_cb(self, waypoints):
        if self.base_waypoints is None:
            self.base_waypoints = copy.deepcopy(waypoints.waypoints)
            self.frame_id = waypoints.header.frame_id

    def traffic_cb(self, msg):
        self.traffic_light_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
        
    def distance_endpoints(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)
    
    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
    
    def decelerate(self, waypoints, red_light_idx):
        last = waypoints[red_light_idx]
        last.twist.twist.linear.x = 0.
        vel = 0.
        for idx, wp in enumerate(waypoints):
            if idx > red_light_idx:
                vel = 0.
            else:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
                vel = math.sqrt(2 * MAX_DECEL * dist)
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')