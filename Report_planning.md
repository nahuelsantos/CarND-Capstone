# Planning

The `waypoint_updater` node will publish future waypoints from current position. It would also modify the speed given the traffic light condition. If no red light has been detected, this node will set the future waypoints' speed to be a constant number. As one will see in the simulator, the speed would be around 15 MPH when no traffic is detected. However when traffic light is detected and the right stopping region is entered, the future waypoints' speed would be reduced in a linear fashion.

## Setup
The `waypoint_updater` node is subscribed to three topics. Namely, they each are the `/current_pose`, `/base_waypoints`, `/traffic_waypoint`. 

```python
	rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
	rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
```

And it is publishing the `\finaly_waypoints` topic to be send to the controller. 
```python
	self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
```

From the subscribers' side it will fetch the entire track, the vehicle's current position and the location of the traffic light.

The publisher would then make use of this information to compose future waypoints that are sent to the controller.

### User defined parameters

Beside the default `LOOKAHEAD_WPS`, the user can set a region where the vehicle is suppose to stop when red light is seen. They may adjust this with the `MIN_D` and `MAX_D`.

The `RefSpeed` defines each target waypoint's speed without the inteference of traffic. The `STOP_LINE` acts as a cutoff region where the vehicle must be absolutely stopped when seen red light.

The `BufferTime` defines a stale time for the vehicle to react to traffic light.  

## Future Waypoints

In side the `loop`, the waypoint_update node will continusly fetch an index from the `/base_waypoints` topic that is closely related the current position of the vehicle. 

```python
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
```
This is then used to fetch next number of `LOOKAHEAD_WPS` and store them in a `lookAheadWpts` list, which will be published to the final waypoint topic `/final_waypoints`. 
```python
    def get_future_wpts(self):
        # get index closest to current position
        self.last_wp = self.nearest_wp(self.last_pos.position, self.base_waypoints)+1
        # fetch next LOOKAHEAD number of waypoints
        ahead = min(len(self.base_waypoints),self.last_wp+LOOKAHEAD_WPS)
        # deep copy a set of lookahead pts
        lookAheadWpts = deepcopy(self.base_waypoints[self.last_wp:ahead])
        # construct default speed for lookAheadWpts
        for waypoint in lookAheadWpts:
            waypoint.twist.twist.linear.x = RefSpeed
```

## Incorporating Traffic Light

To decide when to slow down, there are two conditions to consider. One condition decides if the traffic light is truly ahead of the vehicle and in the region for slowing down. Another condition decides whether the traffic light is new by looking at its timestamp and comparing it against a `BufferTime` variable. 

```python
    def decides_to_stop(self):
        # use two conditions to determine when to slow down and when to go full throttle
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

        if legit_ahead == True and new_traffic == True:
            return True
        else:
            return False
```
Once the vehicle figures out it must slow down, it will asign each corresponding future waypoint a new but smaller speed than the reference speed `RefSpeed`.
```python
    def speed_before_traffic(self, d_car_light):
        """Return waypoint speed when traffic light is seen"""
        # speed = 0.0
        if d_car_light < MIN_D:
            speed = 0.0
        elif d_car_light < MAX_D:
            speed = (RefSpeed/2) * ((d_car_light - MIN_D) / (MAX_D - MIN_D))
        
        return speed
```
And finally it will construct message to be send to the final waypoint topic.
```python
    def construct_msg(self, lookAheadWpts):
        # construct message to be sent
        message_to_sent = Lane()
        message_to_sent.header.stamp = rospy.Time.now()
        message_to_sent.header.frame_id = self.frame_id
        message_to_sent.waypoints = lookAheadWpts
        return message_to_sent
```