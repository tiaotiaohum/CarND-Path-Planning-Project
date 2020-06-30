# Model Building

## Trajectory connection
### Smooth conncetion
I used the previous waypoints, which are not processed to calculate the angle orientation for a smooth connection.
### Waypoints generation
I used the frenet cooridinates to plan the grob waypoints in 30m, 60m, 90m. Then based on the lane number to generate a rough spline.

The lane number is a variable depends on the traffic environment analysis, which will be discussed later on.

#### For Speed Control
Then I divided the first 30m of the rough spline to N fine waypoints, based on the speed, because every 0.02m, the vehicle will visite each way points. 
The number N is essential for teh speed control, and can be calculated as 
>N = 30/(0.02*reference_velocity/2.24)


#### For Longitudianl Acceleration and Jerk Control
for the reference_velocity, I built up a PID control with only P portion. And the P value is relevant for the acceleration and jerk in the longitudinal direction. According to the simulation, the acceleration is overall less then 5m/s*s.

## Traffic Environment Analysis

Because all of the vehicle positions in the map are given by the sensor_fusion variable.

If a slow vehicle is 20m ahead of ego vechile, the vehicle begins to decelated itself.

If no around the vechile in the ajacent lanes the ego vehicle(30m behind, and 15m afront the ego vehicle) can change to the free lane. In the implementation, left lane has higher priority then lane. 



