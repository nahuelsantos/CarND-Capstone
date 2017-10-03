#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
# User defined constraint
BufferTime = 1.1 # when seen traffic light, time to react, in seconds
RefSpeed = 10 # set full throttle speed to 10 mph
MIN_D = 25 # minimum distance before reaching the traffic light
MAX_D = 38 # maximum distance before reaching the traffic light

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_pos = None
        self.base_waypoints = None
        self.last_wp = None
        self.frame_id = None
        self.traffic_light_index = -1
        self.traffic_light_time = rospy.get_time()

        self.loop()
        # rospy.spin()

    def loop(self):
        # published final wayponts

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
            if self.base_waypoints is not None and self.last_pos is not None:
                # get index closest to current position
                self.last_wp = self.nearest_wp(self.last_pos.position, self.base_waypoints)+1
                # fetch next LOOKAHEAD number of waypoints
                ahead = min(len(self.base_waypoints),self.last_wp+LOOKAHEAD_WPS)
                lookAheadWpts = self.base_waypoints[self.last_wp:ahead]
                # construct default speed for lookAheadWpts
                # for waypoint in lookAheadWpts:
                #     waypoint.twist.twist.linear.x = RefSpeed

                # considers the traffic light position
                # use two conditions to determine when to slow down and when to go full throttle
                if self.traffic_light_index != None and self.traffic_light_index > -1:
                    legit_ahead = False
                    new_traffic = False

                    # when traffic light is first seen
                    if self.traffic_light_time > rospy.get_time() - BufferTime:
                        new_traffic = True
                    # when traffic light is ahead
                    if self.traffic_light_index > self.last_wp:
                        # calculate the distance between car and the traffic_light
                        d_car_light = self.distance(self.base_waypoints, self.last_wp, self.traffic_light_index)
                        # determine if this distance falls within a suitable range
                        if d_car_light > MIN_D and d_car_light < MAX_D:
                            legit_ahead = True

                    # when traffic light is first seen and is ahead
                    if new_traffic == True and legit_ahead == True:
                        # slow down gradually
                        for index, waypoint in enumerate(lookAheadWpts):
                            wp_vel = self.get_waypoint_velocity(waypoint)
                            wp_traffic_d = self.distance(self.base_waypoints, index, self.traffic_light_index)
                            if wp_traffic_d > MIN_D + 5: # Added some random number so it start braking ahead
                                # velocity does some linear degration 
                                new_wp_vel = wp_vel*(wp_traffic_d - MIN_D) / (MAX_D - MIN_D)
                                waypoint.twist.twist.linear.x = new_wp_vel


                # construct message to be sent
                message_to_sent = Lane()
                message_to_sent.header.stamp = rospy.Time.now()
                message_to_sent.header.frame_id = self.frame_id
                message_to_sent.waypoints = lookAheadWpts
                self.final_waypoints_pub.publish(message_to_sent)

    def pose_cb(self, msg):
        # TODO: Implement
        self.last_pos = msg.pose
        self.frame_id = msg.header.frame_id

       
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

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        """Store the map data"""
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_index = msg.data
        self.traffic_light_time = rospy.get_time()

    def obstacle_cb(self, msg):
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
