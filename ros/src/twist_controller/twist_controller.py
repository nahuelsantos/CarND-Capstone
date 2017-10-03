#!/usr/bin/env python

import rospy
from pid import *
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg   import Float32


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
PID_CONTROL_RESET_Trend_CH = False
PID_CONTROL_RESET_Target_CH = False
PID_STEER_RESET_Trend_CH = True
PID_STEER_RESET_Target_CH = False
CALIBRATION_LOG = False

class Controller(object):
    def __init__(self, *args, **kwargs):
        
        # Define the 2 PIDs: one for the throttle/brake control, the second one for the steering
        self.pid_control = PID(3, .5, .125)#, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
        # PID Steer Ku --> 2.5 Tu --> 0.6(30samples at 0.02s) 
        # self.pid_steering = PID(.375 , 1.25, .028, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        self.pid_steering = PID(.475 , 1.25, .028, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        
        #self.pid_steering = PID(.6 , 1.2, .02, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        ####### PARAMETERS coming from Zeigler Nichols analisys
        #self.pid_steering = PID(.6 , 1.2, .06, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])

        # Define the low pass filter to be applied to steering error value
        self.steer_error_lpf = LowPassFilter(.5, .1)
        self.steer_lpf = LowPassFilter(.5, .1)

        self.yaw_controller = YawController(kwargs["wheel_base"], kwargs["steer_ratio"], kwargs["min_speed"], kwargs["max_lat_accel"], kwargs["max_steer_angle"])
        self.brake_deadband = kwargs["brake_deadband"]
        self.time = None

        self.last_speed_target = 0.0
        self.last_steer_target = 0.0


    def control(self, *args, **kwargs):

        try:
            twist = kwargs['twist_cmd']
            current_velocity = kwargs['current_vel']
        
            # get current/target velocities (linear and angular) from the received messages in the topics 		
            target_lin_vel = twist.twist.linear.x
            target_ang_vel = twist.twist.angular.z
                    
            current_lin_vel = current_velocity.twist.linear.x
            current_ang_vel = current_velocity.twist.angular.z
        
            # convert angular speed (current and target) to steering angle (current and target)
            current_steer = self.yaw_controller.get_steering(current_lin_vel, current_ang_vel, current_lin_vel)
            target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)		

            # Used to reset PIDs integral component depending on the target change
            self.check_targets_for_reset(target_lin_vel, target_steer)

            current_time = rospy.get_time()

            if(self.time != None):
                delta_t = current_time - self.time + 1e-6
                
                # Manage Throttle and Brake using a single PID and considering deadband value too
                speed_err = target_lin_vel - current_lin_vel
                throttle_brake = self.pid_control.step(speed_err, delta_t)

                throttle = max(0.0,throttle_brake)
                brake = max(0.0, -throttle_brake)
                if(brake < self.brake_deadband):
                    brake = 0.0
            
                # Manage Steer using the dedicated PID
                # steer_err_rough = target_steer - current_steer
                # steer_err = self.steer_error_lpf.filt(steer_err_rough)
                steer_err_rough = target_steer - self.steer_error_lpf.filt(current_steer)
                steer_err = steer_err_rough

                if(CALIBRATION_LOG):
                    self.pid_steering.log(steer_err, delta_t, "steer")
                
                steer_rough = self.pid_steering.step(steer_err, delta_t)
                steer = self.steer_lpf.filt(steer_rough)
                
                if(CALIBRATION_LOG):
                    rospy.loginfo('SpeedCurrent -> %f, SpeedTarget -> %f, SteerCurrent -> %f, SteerTarget -> %f', current_lin_vel, target_lin_vel, current_steer, target_steer)
                    rospy.loginfo('Throttle_brake -> %f, Throttle -> %f, Brake -> %f, Steer -> %f, Steer_rough -> %f, error -> %f, error_rough -> %f ', throttle_brake, throttle, brake, steer, steer_rough, steer_err, steer_err_rough)            

                self.time = current_time

                return throttle, brake, steer
            else:
                self.time = current_time
                return 0.0, 0.0, 0.0
        except Exception, e:
            print(e)

    def check_targets_for_reset(self, target_lin_vel, target_steer):
        #################################
        # Reset pid_control integral part
        #################################
         
         # Target Change control PID reset
        if(PID_CONTROL_RESET_Target_CH and self.last_speed_target != target_lin_vel):
            self.pid_control.reset()

        # Trend change control PID reset
        if(PID_CONTROL_RESET_Trend_CH and ((target_lin_vel > self.last_speed_target and self.pid_control.int_val < 0) or (target_lin_vel < self.last_speed_target and self.pid_control.int_val > 0))):
            self.pid_control.reset()
            #rospy.logwarn("PID RESETTED")

        self.last_speed_target = target_lin_vel

        #################################
        # Reset pid_steer integral part
        #################################
        # Target Change steer PID reset
        if(PID_STEER_RESET_Target_CH and self.last_steer_target != target_steer):
            self.pid_steering.reset()

        # Trend change control PID reset
        if(PID_STEER_RESET_Trend_CH and ((target_steer > self.last_steer_target and self.pid_steering.int_val < 0) or (target_steer < self.last_steer_target and self.pid_steering.int_val > 0))):
            self.pid_steering.int_val /= 2
            #self.pid_steering.reset()
            #rospy.logwarn("PID RESETTED")

        self.last_steer_target = target_steer

    def reset(self):
        self.pid_control.reset()
        self.pid_steering.reset()
