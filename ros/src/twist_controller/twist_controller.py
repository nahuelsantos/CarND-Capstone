#!/usr/bin/env python

import rospy
from pid import *
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg   import Float32
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

PID_CONTROL_RESET_Trend_CH_EN = True
PID_CONTROL_RESET_Target_CH_EN = False
PID_STEER_RESET_Target_CH_EN = False
PID_STEER_RESET_Trend_CH_EN = True
PID_CONTROL_MIN_RESET_EN = False
PID_CONTROL_MIN_RESET_TH = .5
CALIBRATION_PARAMS = False
CALIBRATION_LOG = False
USE_PID_FOR_STEERING = False
BRAKE_FACTOR = .6
PID2BRAKE_ADJ =  .3
BRAKE_MIN = -.04
THROTTLE_MAX = .75
THROTTLE_MIN = .07
THROTTLE_MAX_CHANGE = .06
PID2THROTTLE_ADJ =  .6

class Controller(object):
    def __init__(self, *args, **kwargs):
        
        
        ############# Define the 2 PIDs: one for the throttle/brake control, the second one for the steering
        self.pid_control = PID(1.8, .005, .0)#, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
        #self.pid_control = PID(1, .1, .075, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
        #self.pid_control = PID(5, .45, .125, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
        ####### PARAMETERS coming from Zeigler Nichols analisys
        #self.pid_steering = PID(.6 , 1.2, .06, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        # PID Steer Ku --> 2.5 Tu --> 0.6(30samples at 0.02s) 
        self.pid_steering = PID(.70 , 1.2, .0282, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        
        ####### Define the low pass filter to be applied to steering target value
        self.steer_error_lpf = LowPassFilter(.4, .1) # Not Used in the current implementation
        self.steer_lpf = LowPassFilter(.5, .1) # Not Used in the current implementation
        self.steer_target_lpf = LowPassFilter(.3, .1)
        self.control_error_lpf = LowPassFilter(.45, .5)

        self.max_steer_angle = kwargs["max_steer_angle"]
        self.yaw_controller = YawController(kwargs["wheel_base"], kwargs["steer_ratio"], kwargs["min_speed"], kwargs["max_lat_accel"], self.max_steer_angle)

        self.brake_deadband = kwargs["brake_deadband"]
        self.time = None

        self.last_speed_target = 0.0
        self.last_steer_target = 0.0
        self.last_throttle = 0.0
        vehicle_mass = float(kwargs["vehicle_mass"]) + float(kwargs["fuel_capacity"]) * GAS_DENSITY
        self.max_brake_torque   = BRAKE_FACTOR * vehicle_mass * abs(float(kwargs["decel_limit"])) * float(kwargs["wheel_radius"])
        self.maxRefSpeed = float(rospy.get_param('/waypoint_loader/velocity'))*1000/3600        

    def control(self, *args, **kwargs):

        try:
            current_time = rospy.get_time()

            twist = kwargs['twist_cmd']
            current_velocity = kwargs['current_vel']
        
            ####### Get current/target velocities (linear and angular) from the received messages in the topics 		
            target_lin_vel = twist.twist.linear.x
            target_ang_vel = twist.twist.angular.z
            current_lin_vel = current_velocity.twist.linear.x
            current_ang_vel = current_velocity.twist.angular.z
            ####### Convert angular speed (current and target) to steering angle (current and target)
            current_steer = self.yaw_controller.get_steering(current_lin_vel, current_ang_vel, current_lin_vel)
            target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
            target_steer = self.steer_target_lpf.filt(target_steer)
            
            #target_steer = max(-self.max_steer_angle,min(self.max_steer_angle,target_steer))
            ####### Used to reset PIDs integral component depending on the target change
            self.check_targets_for_reset(target_lin_vel, target_steer)

            if(self.time != None):
                delta_t = current_time - self.time
                #### Manage Throttle and Brake using a single PID and considering deadband value too
                speed_err = target_lin_vel - current_lin_vel
                #speed_err = self.control_error_lpf.filt(target_lin_vel - current_lin_vel)
                throttle_brake = self.pid_control.step(speed_err, delta_t)
                
                # PID output > 0 means the car need to accelerate
                if throttle_brake >= 0.0:
                    throttle = max(0.0,throttle_brake*PID2THROTTLE_ADJ)
                    throttle = self.convertToActuation(throttle_brake, THROTTLE_MAX)
                    if throttle - self.last_throttle > THROTTLE_MAX_CHANGE:
                        throttle = self.last_throttle + THROTTLE_MAX_CHANGE
                        self.last_throttle = throttle
                    if throttle < THROTTLE_MIN:
                        throttle = 0.0
                    brake = 0.0
                # BRAKE_MIN < PID output < 0 means the car can avoid to brake but need to stop accelerating
                elif throttle_brake >= BRAKE_MIN:
                    throttle = self.last_throttle = 0.0
                    brake = 0.0
                # PID output < BRAKE_MIN  means the car need to decelerate
                else:
                    throttle = self.last_throttle = 0.0
                    brake = max(0.0, -throttle_brake*PID2BRAKE_ADJ)
                    if(brake < self.brake_deadband):
                        brake = 0.0
                    else:
                        brake = self.convertToActuation(brake, self.max_brake_torque) 
                
                if USE_PID_FOR_STEERING:
                    #### Manage Steer using the dedicated PID
                    current_steer_filt = self.steer_error_lpf.filt(current_steer) # Not Used in the current implementation
                    steer_err_rough = target_steer - current_steer_filt # Not Used in the current implementation
                    steer_err = steer_err_rough # Not Used in the current implementation

                    if(CALIBRATION_LOG): # Not Used in the current implementation
                        self.pid_steering.log(steer_err, delta_t, "steer") # Not Used in the current implementation
                    
                    steer_rough = self.pid_steering.step(steer_err, delta_t) # Not Used in the current implementation
                    steer = 0
                    if (throttle != 0 or target_lin_vel > 0.5):
                        steer = self.steer_lpf.filt(steer_rough) # Not Used in the current implementation
                    else:
                        self.pid_steering.reset()
                    # steer_err_rough = target_steer - self.steer_error_lpf.filt(current_steer)
                
                if(CALIBRATION_LOG): 
                    rospy.loginfo('SpeedCurrent -> %f, SpeedTarget -> %f, SteerCurrent -> %f, SteerTarget -> %f, SteerCurrentFilt -> %f, Target_angular_speed -> %f', 
                                   current_lin_vel, target_lin_vel, current_steer, target_steer, current_steer_filt, target_ang_vel) # Not Used in the current implementation

                    rospy.loginfo('Throttle_brake -> %f, Throttle -> %f, Brake -> %f, Steer -> %f, Steer_rough -> %f, error -> %f, error_rough -> %f ', 
                                   throttle_brake, throttle, brake, steer, steer_rough, steer_err, target_steer - current_steer ) # Not Used in the current implementation

                self.time = current_time
                if(self.maxRefSpeed - current_lin_vel < 0):
                    print("************ MAX SPEED SURPASSED ************")
                    print("Target: {} - Current:{} ".format(self.maxRefSpeed,current_lin_vel))
                if USE_PID_FOR_STEERING:
                    return throttle, brake, steer
                else:
                    return throttle, brake, target_steer
            else:
                self.time = current_time
                return 0.0, 0.0, 0.0
        except Exception, e:
            print(e)

    def check_targets_for_reset(self, target_lin_vel, target_steer):

        #################################
        # Reset pid_control if target is smaller than a certain value
        #################################
        
        if(PID_CONTROL_MIN_RESET_EN and target_lin_vel < PID_CONTROL_MIN_RESET_TH):
            self.pid_control.reset()

        #################################
        # Reset pid_control integral part
        #################################
         
         # Target Change control PID reset
        if(PID_CONTROL_RESET_Target_CH_EN and self.last_speed_target != target_lin_vel):
            self.pid_control.reset()

        # Trend change control PID reset
        if(PID_CONTROL_RESET_Trend_CH_EN and ((target_lin_vel > self.last_speed_target and self.pid_control.int_val < 0) or (target_lin_vel < self.last_speed_target and self.pid_control.int_val > 0))):
            self.pid_control.reset()

        self.last_speed_target = target_lin_vel

        #################################
        # Reset pid_steer integral part
        #################################
        # Target Change steer PID reset
        if(PID_STEER_RESET_Target_CH_EN and self.last_steer_target != target_steer):
            self.pid_steering.reset()

        # Trend change control PID reset
        if(PID_STEER_RESET_Trend_CH_EN and ((target_steer > self.last_steer_target and self.pid_steering.int_val < 0) or (target_steer < self.last_steer_target and self.pid_steering.int_val > 0))):
            self.pid_steering.int_val /= 2
            #self.pid_steering.reset()

        self.last_steer_target = target_steer

    def reset(self):
        self.pid_control.reset()
        self.pid_steering.reset()
        
    def convertToActuation(self, value, scale):
        return scale * math.tanh(value)