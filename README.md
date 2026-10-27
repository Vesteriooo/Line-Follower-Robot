# Line-Follower-Robot
Code for a shortest path left-hand priority line follower robot

Features of the Program:
- PID Realignment Algorithm
  - The Program uses Proportional-Integral-Derivative Algorithm to calculate how much the robot will turn to realign itself towards the center whenever it sways away from it.
- Left-Hand Priority
  - The turning decision of the robot will follow a rule of which turn it will take depending on the variables available. Left-Straight-Right-UTurn
- Active Brakes
  - When turning the Brakes will turn the opposite direction whenever it needs to stop, instead of just stopping the wheels from turning, to avoid overshooting.
- Path Pruning
  - Will automatically calculate the shortest path it will take from start to finish.
- Hard Start Lock
  - The robot will not run in "Shortest-Path" mode if the scanning hasn't been acomplished.
- Active Sweep
  - Prevents the robot to mis-read junctions if it's slightly off center.
- Anti-Blindfold Validation
  - Prevents the robot to re-read the same line whenever it turns.


Hardware Used: 
- ESP32
- L298N Motor Driver
- 2X TT DC Gear Motor
- 5X Individual IR Sensors
- TCS34725 RGB Color Sensor


NOTE: Most of the variables needs changing due to difference of robot size, and positioning of sensors-to-wheel ratio.
I would not recommend changing the Pins, but you do you.
I've put comments of what can and cannot be changed for the variables.
If you'll change the base_speed and turn_speed, it is recommended that you should also change the turning variables.

**IMPORTANT REMINDER**
Not every robot is the same, variables will be changed almost 100% of the time
But the code is working perfectly, you just need to properly adjust the variables to fit your robot

The 3 Middle sensors should be in line which each other, while the 2 outer sensors should be behind a few (2-6) mm behind
This is to ensure that the turning algorithm will work perfectly
