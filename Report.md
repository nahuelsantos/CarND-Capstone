# The Controller

Each cycle of the main loop, the *controller* has the goal to compute the actuation to let the car follow the target longitudinal and angular speeds required by the *supervisor*. These target information, together with the current angular and longitudinal speeds, are defined in the messages transferred via __/current\_velocity__, and __/twist\_cmd__ topics by the __waypoint\_updater__. These values could change every iteration of the loop and the controller should satisfy the requests.

In addition, in order to let the system to be used also in cases where the driver wants to come back to the *manual* driving, the messages from the __/vehicle/dbw_enabled__ has to be considered. This message specify if the driver is currently driving the car by its own and the system need to be turned off and resetted. 

## The Ros interface for the controller

The *dbw_node* defines the interface between the controller and the ros environment. The main purpose of this object is to communicate with all the other ros nodes (throw topics), and execute the controller step procedure to compute the actuations to be requested to the actuators. 
As first step, all the communication channels are opened (as subscriber for some topics and publisher for others) and the controller is initialized as well. 

```python
self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

params = {"wheel_base": wheel_base, "steer_ratio": steer_ratio, "min_speed": min_speed, "max_lat_accel": max_lat_accel, "max_steer_angle": max_steer_angle, "brake_deadband": brake_deadband, "accel_limit": accel_limit, "decel_limit": decel_limit}
self.controller = Controller(**params)
```

Then the node enters its loop procedure where the controller step is executed, and the resulting commands published in the relative topics.
Moreover, as mentioned before, the *dbw_node* also need to take into consideration the *dbw_enable* signal. In case the signal is True (means the driver is not driving the car manually), the controller has to be executed. Otherwise, the controller has to be resetted, and no actuation need to be computed

 ```python
if(self.dbw_en):
    parameters = {'twist_cmd': self.twist_cmd,
                  'current_vel': self.current_velocity}
    
    throttle, brake, steer = self.controller.control(**parameters)

    self.publish(throttle, brake, steer)
else:
    self.controller.reset()
```

## The controller

In order to satisfy the request for longitudinal and angular speeds, two different PID controllers were initally used. 

### PIDs implementation

Even if the actuations required by the system were three (throttle, brake and steer), because of the strict connection between the throttle and the brake actuations, we decided to define a single PID both for throttle and brake.

```python
# Define the 2 PIDs: one for the throttle/brake control, the second one for the steering
self.pid_control = PID(5, .45, .125)#, mn = kwargs["decel_limit"], mx = kwargs["accel_limit"])
self.pid_steering = PID(.75 , 1.5, .0332, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
```

At each execution of the PIDs control, starting from the current and target values read from the topics, the actuation value is computed using all the three PID components (proportional, integral and derivative).

While the longitudinal speed values appears to be quite usable from the start, the target and current angular speed values required some more refinements. 
1. First, the actuation degree of freedom we have for angular speed is the steering angle of the vehicle;
2. Second, the shared current values for angular speed appears to be quite noisy.

For this reason, both the target and current angular velocity were converted from angular speed to steering value throw the *yaw_controller* utility. 

```python
# get current/target velocities (linear and angular) from the received messages in the topics 		
target_lin_vel = twist.twist.linear.x
target_ang_vel = twist.twist.angular.z

current_lin_vel = current_velocity.twist.linear.x
current_ang_vel = current_velocity.twist.angular.z

# convert angular speed (current and target) to steering angle (current and target)
current_steer = self.yaw_controller.get_steering(current_lin_vel, current_ang_vel, current_lin_vel)
target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
```
For the noise, a preliminary analisys on the used signals was done both for the angular and longitudinal speed information retrieved from the topics. 
While the logitudinal signal has appeared to be quite stable (both for the target and the current values), the current steer (transformation of the current angular speed information) appears to be quite noisy. 

Since the PID controller for steering is directly influenced by the current steer (the input to the PID is exactly the steering error *self.pid_steering.step(steer_err, delta_t)*), to the current steer value was applied a *low_pass* filter. 

Moreover, in order to avoid jerky steer maneuver due to high steer error or to the process noise, a filter is applied also to the output value of the PID itself.

The filters are firstly defined in the initialization stage of the controller itself, and then applied for each step of the controller.

```python
# Define the low pass filter to be applied to steering error value
self.steer_error_lpf = LowPassFilter(.7, .1)
self.steer_lpf = LowPassFilter(.55, .1)

...

current_steer_filt = self.steer_error_lpf.filt(current_steer)
steer_err = target_steer - current_steer_filt

...

steer_rough = self.pid_steering.step(steer_err, delta_t)
steer = self.steer_lpf.filt(steer_rough)
```

Once the values are transformed in the right form, the PIDs step function is executed and the actuation trasferred to the *dbw_node* for publishing.
```python

speed_err = target_lin_vel - current_lin_vel
throttle_brake = self.pid_control.step(speed_err, delta_t)

throttle = max(0.0,throttle_brake)
brake = max(0.0, -throttle_brake)
if(brake < self.brake_deadband):
    brake = 0.0

# Manage Steer using the dedicated PID
current_steer_filt = self.steer_error_lpf.filt(current_steer)
steer_err_rough = target_steer - current_steer_filt
steer_err = steer_err_rough

...

steer_rough = self.pid_steering.step(steer_err, delta_t)
steer = self.steer_lpf.filt(steer_rough)

...

return throttle, brake, steer
```
### Controllers Calibration

In order to have good performances with the PID controllers, a parameter tuning is required. In particular need to be defined all the three gain used by the PID for the proportional, for the integral and for the derivative components. 

The best approach that can be applied in such a calibration, is to define and modelize the system we are trying to control and then analitically compute the controller coefficients. 

Since in this case some of the required information for such a modelization are not known, we decided to approach the calibration with an experimental approach. The well known _*Zeigler-Nichols*_ method was applied, and then a manual refinement was executed.

We first compute the *Zeigler-Nichols* calibration for the speed controller, fixing the steer angle to some different values (0, +-0.1, +-0.05). 
Then the *Zeigler-Nichols* calibration for the steering PID - using the previously found calibration for the speed PID with a fixed target speed - verifing the critical proportional gain with several target steering angles (0, +-0.1, +-0.05).

## Steering management adjustment

After several trial, we figure out we were using a PID controller for something directly controllable. Differently from the longitudinal speed, that is connected to the throttle and the breaking pedals by all the physics of the car, the angular speed is directly related to the steering angle (that is exactly the actuator we are controlling).

For this reason we finally decided to remove the pid for steering controller, and to apply the target steer angle (converted from the angular speed via the *yaw_controller*) at the steer.
```

Once the values are transformed in the right form, the PIDs step function is executed and the actuation trasferred to the *dbw_node* for publishing.
```python

target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
# target_steer = self.steer_target_lpf.filt(target_steer)
target_steer = max(-self.max_steer_angle,min(self.max_steer_angle,target_steer))

...

#return throttle, brake, steer
return throttle, brake, target_steer
```
