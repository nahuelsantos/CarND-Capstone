#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
BUFFER_MAX_DIST = 40.0
BUFFER_MIN_DIST = 3

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.last_pose = None
        self.num_waypoints = 0
        self.closest_waypoint = 0
        self.next_red_light = None
        self.ever_received_traffic_waypoint = False
        self.RefSpeed = 4.4

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        next_waypoints = []
        if (self.last_pose and self.base_waypoints):
            #ix = self.next_waypoint(self.base_waypoints, self.last_pose.pose)
            #self.closest_waypoint = ix
            closest_wpt_tmp = self.nearest_wp(self.last_pose.pose.position, self.base_waypoints)
            closest_wpt = self.adjust_closest_wpt(closest_wpt_tmp, self.base_waypoints, self.last_pose.pose)
            self.closest_waypoint = closest_wpt

            for i in range(self.closest_waypoint, min(self.closest_waypoint+LOOKAHEAD_WPS, len(self.base_waypoints))):
                wpt_target_velocity = self.calculate_waypoint_velocity(i)
                self.base_waypoints[i].twist.twist.linear.x = wpt_target_velocity
                wp = self.base_waypoints[i]
                next_waypoints.append(wp)
            rospy.loginfo("""Waypoint ix: {} num: {}  v: {}""".format(self.closest_waypoint, len(next_waypoints), self.get_waypoint_velocity(next_waypoints[0])))

        final_wps = Lane()
        final_wps.waypoints = next_waypoints

        self.final_waypoints_pub.publish(final_wps)

    def nearest_wp(self, last_position, waypoints):
        """find nearest waypoint index to the current location"""
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        nearest_distance = 9999;
        nearest_index = -1;
        for index, waypoint in enumerate(waypoints):
            waypoint_pos = waypoint.pose.pose.position
            distance = dl(last_position, waypoint_pos)
            if distance < nearest_distance:
                nearest_index = index
                nearest_distance = distance
        return nearest_index

    def adjust_closest_wpt(self, closest_wpt, waypoints, current_pose):

        quaternion = (current_pose.orientation.x,
                      current_pose.orientation.y,
                      current_pose.orientation.z,
                      current_pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        pose_yaw = euler[2]

        if (pose_yaw > (math.pi/4)):
            return closest_wpt + 1

        return closest_wpt

    def calculate_waypoint_velocity(self, waypoint_index):
        if self.ever_received_traffic_waypoint and self.base_waypoints:
            velocity = self.RefSpeed
        else:
            rospy.loginfo('Waiting for waypoints or red-light info, so set zero target velocity.')
            velocity = 0

        if self.next_red_light and self.base_waypoints and (waypoint_index <= self.next_red_light):
            distance_to_red_light = self.distance(self.base_waypoints, waypoint_index, self.next_red_light)
            if (distance_to_red_light < BUFFER_MIN_DIST):
                velocity = 0.0
            elif (distance_to_red_light < BUFFER_MAX_DIST):
                ratio = distance_to_red_light / BUFFER_MAX_DIST
                velocity = self.RefSpeed * ratio

        if velocity > self.RefSpeed:
            velocity = self.RefSpeed

        return velocity

    def pose_cb(self, msg):
        self.last_pose = msg

    def waypoints_cb(self, waypoints):
        if(not self.base_waypoints):
            self.base_waypoints = waypoints.waypoints
        #self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        if (msg.data >= 0):
            self.next_red_light = msg.data
        else:
            self.next_red_light = None
        self.ever_received_traffic_waypoint = True

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')