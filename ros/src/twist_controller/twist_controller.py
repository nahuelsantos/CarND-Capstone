from pid import *
from yaw_controller import YawController
from math import sqrt, cos, sin
import numpy as np
import tf
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

def get_cte(pose, wpts_ahead):
    # transform wpts_ahead coordinates into cur_pos's frame
    saved_x = []
    saved_y = []
    # save yaw angle
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                                 pose.orientation.y,
                                                                 pose.orientation.z,
                                                                 pose.orientation.w])
    # save starting point
    x_origin = pose.position.x
    y_origin = pose.position.y
    # transform
    for i in range(8):
        # translational
        tempx = wpts_ahead[i].pose.pose.position.x - x_origin
        tempy = wpts_ahead[i].pose.pose.position.y - y_origin
        # rotate
        final_x = tempx * cos(yaw) + tempy * sin(yaw)
        final_y = -tempx * sin(yaw) + tempy * cos(yaw)
        # save
        saved_x.append(final_x)
        saved_y.append(final_y)
    # polyfit
    coeffs = np.polyfit(saved_x, saved_y, 3)
    # eval some distance ahead
    cte_dist = np.polyval(coeffs, 4.0)
    return cte_dist

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.kp = args[0]
        self.ki = args[1]
        self.kd = args[2]
        
        # controller parameters
        self.accel_limit = args[3]
        self.decel_limit = args[4]
        self.max_steer_angle = args[5]
        self.vehicle_mass = args[6]
        self.radius_wheel = args[7]
        self.wheel_base = args[8]
        self.steer_ratio = args[9]
        self.max_lat_accel = args[10]
        self.min_speed = 0.0 # some random min speed I made up 
        self.time_i = rospy.get_time()

        # define three controller
        # controller for throtting and braking
        self.accel_controller = PID(self.kp, self.ki, self.kd, 
                                    mn=self.decel_limit, mx=self.accel_limit)
        # controller for off road steering
        self.steer_controller = PID(self.kp, self.ki, self.kd,
                                    mn=-self.max_steer_angle, mx=self.max_steer_angle)
        # controller for on road steering
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 
                                            self.min_speed, self.max_lat_accel,
                                            self.max_steer_angle)
        
    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_vel = args[0]
        target_ang = args[1]
        current_vel = args[2]
        current_pos = args[3]
        dbw_en = args[4]
        wpts_ahead = args[5]
        # fetch current time and update
        time_now = rospy.get_time()
        sample_time = time_now - self.time_i + 1e-6
        self.time_i = time_now

        # when manual driving is enabled, reset the controller 
        if not dbw_en:
            self.accel_controller.reset()
            self.steer_controller.reset()
            
        # find cross track error between current position and the intented trajectory
        cte = get_cte(current_pos, wpts_ahead)
        rospy.logwarn("cte: %s", cte)
        # find velocity magnitude difference
        tar_vel_mag = target_vel.x
        cur_vel_mag = current_vel.x        
        vel_diff = tar_vel_mag - cur_vel_mag
        # in case of off road, find control input that put the vehicle back on the track
        steer = self.steer_controller.step(cte, sample_time)
        rospy.logwarn("steer: %s", steer)
        # predictive steering
        steer_2 = self.yaw_controller.get_steering(tar_vel_mag, target_ang.z, cur_vel_mag)
        # addup for actual steer
        steer = steer + steer_2
        # find throttle and brake
        accel = self.accel_controller.step(vel_diff, sample_time)
        if accel >= 0:
            throttle = accel
            brake = 0
        else:
            # brake should be calculated with desired acceleration, weight
            # and wheel radius: F = ma, torque = F*radius_wheel
            throttle = 0;
            force = accel*self.vehicle_mass
            brake = force*self.radius_wheel

        return throttle, brake, steer
