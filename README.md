# Trail-tracking-MPC-APF
This is a MATLAB project aiming to realize a new trail tracking and obstacle avoidance method
I applied Bezier curve as the track to simulate a situation where a vehicle's initial and terminal directions are determined. You can change three control points to generate your expected tracks like this:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Curve.jpg)
There are three given control points in this figure:(0,0),(30,30/sqrt(3)),(30,30). In this way we can define the initial and final deriction of the virtual vehicle.
Run "My_MPC.m" and observe the output:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Tracking.png)
The virtual vehicle's driving trail is shown by purple spots.
The program also records the change of yawing angle of the whole process like this:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Facing%20angle.png)
