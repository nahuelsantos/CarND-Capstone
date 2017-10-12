#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.has_image = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
        traffic_light_classifier_config = rospy.get_param("~traffic_light_classifier")
        
        # Load the right model depending on the param loaded in the launch
        if traffic_light_classifier_config == "REAL":
            print("Loaded classifier for real world use")
            model_name = "squeezeNet_real_environment"
        if traffic_light_classifier_config == "SIM":
            print("Loaded classifier for simulator use")
            model_name = "squeezeNet_sim_environment"

        self.light_classifier = TLClassifier(model_name)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        rate = rospy.Rate(6)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()
        

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
    def loop(self):

        light_wp, state = self.process_traffic_lights()

        #print("Classified Light State: {}- light WP: {} ".format(state,light_wp))
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

    def distance(self, pose_from, pose_to):
        """Compute the distance between two positions
        Args:
            pose_from: pose from
            pose_to: pose to
        Returns:
            float: euclidean distance
        """
        return math.sqrt((pose_from.position.x - pose_to.position.x)**2 + (pose_from.position.y - pose_to.position.y)**2)

    @staticmethod
    def get_yaw(pose):
        """Return the yaw of the input
        Args:
            pose (Pose): position to compute the yaw
        Returns:
            float: yaw angle
        """
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)

        return euler[2]

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        if (self.waypoints is None):
            rospy.logwarn("tl_detector.get_closest_waypoint :: No waypoints yet")
            return

        min_dist = None
        idx = 0
        wp = None
        for i, w in enumerate(self.waypoints.waypoints):
          dist = self.distance(pose, w.pose.pose)
          if min_dist is None or dist < min_dist:
              min_dist = dist
              idx = i
              wp = w
        heading = math.atan2((w.pose.pose.position.y - pose.position.y),
                             (w.pose.pose.position.x - pose.position.x))
        yaw = self.get_yaw(pose)
        angle = math.fabs(yaw-heading)
        if angle > math.pi/4.:
            idx += 1
            if idx >= len(self.waypoints.waypoints):
                idx = 0
        return idx


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return (0, 0)

        #TODO Use tranform and rotation to calculate 2D position of light in image
        cx = image_width/2
        cy = image_height/2

        rpy = tf.transformations.euler_from_quaternion(rot)
        yaw = rpy[2]

        (ptx, pty, ptz) = (point_in_world.x, point_in_world.y, point_in_world.z)

        point_to_cam = (ptx * math.cos(yaw) - pty * math.sin(yaw),
                        ptx * math.sin(yaw) + pty * math.cos(yaw),
                        ptz)
        point_to_cam = [sum(x) for x in zip(point_to_cam, trans)]

        ##########################################################################################
        # Tweak to get it working on the simulator
        if fx < 10:
            fx = 1400
            fy = 2000
            cx = image_width/2
            cy = image_height
        ##########################################################################################

        x = -point_to_cam[1] * fx / point_to_cam[0];
        y = -point_to_cam[2] * fy / point_to_cam[0];

        x = int(x + cx)
        y = int(y + cy)

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        # margin = 100
        # Corrected to be adapted with the squeeze net configuration
        margin = 112
        cv_image = cv_image[y-margin:y+margin,x-margin:x+margin]

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    @staticmethod
    def create_pose(x, y, z, yaw=0.):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if(car_position):
                #TODO find the closest visible traffic light (if one exists)
                min_dist = None
                idx = 0
                for i, l in enumerate(self.lights):
                    dist = self.distance(self.pose.pose, l.pose.pose)
                    if min_dist is None or dist < min_dist:
                        min_dist = dist
                        idx = i
                lwp = self.get_closest_waypoint(self.lights[idx].pose.pose)
                cwp = self.get_closest_waypoint(self.pose.pose)
                # Car waypoint is greater than light waypoint?
                if lwp <= cwp:
                    idx += 1
                    if idx >= len(self.lights):
                        idx = 0
                if min_dist is not None:
                    light = self.lights[idx]
                    sl_x, sl_y = stop_line_positions[idx]
                    stop_line_pose = self.create_pose(sl_x, sl_y, 0.0)
                    stop_line_wp = self.get_closest_waypoint(stop_line_pose.pose)

        if light:
            state = self.get_light_state(light)
            return stop_line_wp, state
# Comment next line to avoid "cleaning" the waypoints
#        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
