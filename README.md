# Trail-tracking-MPC-APF
This is a MATLAB project aiming to realize a new trail tracking and obstacle avoidance method
I applied Bezier curve as the track to simulate a situation where a vehicle's initial and terminal directions are determined. You can change three control points to generate your expected tracks like this:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Curve.jpg)

There are three given control points in this figure:(0,0),(30,30/sqrt(3)),(30,30). In this way we can define the initial and final deriction of the virtual vehicle.
Run "My_MPC.m" and observe the output:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Trail%20tracking.png)

The virtual vehicle's driving trail is shown by purple spots.
The program also records the change of yawing angle of the whole process like this:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Yawing%20angle.png)

The facing angle changes from 30 to 90 degrees.
Change the initial position, we can find that the vehicle can go back to the reference trail without problem:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Back%20to%20trail.png)

As an innovative application of artificial potential field(AFP), the approch combined the AFP and MPC to get a high quality of obstacle avoidance:
![](https://github.com/fuingcrazy/Trail-tracking-MPC-APF/blob/master/Pictures/Avoidance.png)
