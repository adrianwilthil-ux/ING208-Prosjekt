# ING208-Prosjekt



\--- Overview over programs ---







\--- Main programs ---



MOTION\_SIM.m

Main simulation program. Simulates kinematics and dynamics. Takes in a number of points in ceiling frame, as well as base tilt magnitude and direction. Calculates a full trajectory using different types of trajectories and interpolations, then plots the robot, position+velocity+acceleration graph and torque graph. Displays maximum values for joint speed and torque. Explained in (3.3, 3.4) and demonstrated in (4.2)



REACHABILITY\_CHECKER.m

Verifies our robot dimensions meets requirements. Checks every pose needed within a set resolution and returns the successful/unsuccessful amount. Also checks joint velocities needed for a 10cm/s z-direction speed. Resolution can be specified before running. Outputs amount of reachable and unreachable poses, as well as the maximum speed for each joint to meet the speed criterion and the number of poses which failed it. Explained and demonstrated in (4.1)



link\_length\_calc.m

Outputs maximum and minimum elbow angles for given link lengths. Used to dimension the robot according to specifications. Used in (3.1)



forward\_kinematics.m

Calculates the base-tooltip transform without Toolbox commands. Uses DH parameters to obtain transformation matrices for each frame, then combines them. The transform is then tested against the Toolbox solution. Used in (3.1)



singularities.m

Used to demonstrate that the Jacobian loses rank when robot is in singular configurations. Used in (3.1)



sensor\_transform.m

Transforms a point seen by the camera system and a tilt angle from the inclinometers into a usable pose with drill perpendicular to ceiling. Used in (3.2)



wrist\_torque.m

Tests both wrists required torque for lifting drill from straight down to straight up.



velocity\_controller.m

Creates a PI velocity controller for the model from joint\_dynamics.m. Plots the step response. 



position\_controller.m

Outer loop P position controller utilizing the velocity controller. Plots the step-response.











\--- Configs and helpers ---



config.m

The main config file used by almost all programs. Robot dimensions, masses, CoMs, workspace dimensions, trajectory times, drilling force and moment, etc.



robot\_model.m

Imports dimensions and dynamic values from config.m and stores it in a robot object used by all other programs. Adds gravity. Defines a zero-pose.



joint\_dynamics.m

Uses parameters discussed in (4.4) and creates a joint dynamic model transfer function.



ik5.m (function)

Our custom inverse kinematics solver for a 5DoF arm. Used by all programs which performs inverse kinematics. Was written with AI assistance. Explained in (3.2). 



ceiling\_to\_base.m (function)

Transforms ceiling coordinates (x, y, offset) and tilt magnitude and direction into a pose in the base frame. Used I.E. for simulation input, as finding valid points on the disc for different tilts is quite a job.



plot\_disc.m (function)

Takes in points in ceiling frame, as well as tilt angle and direction, plots points and disc. Used by MOTION\_SIM.m



plot\_dynamics.m (function)

Takes in the simulation results: joint position, velocity and acceleration as well as torques, and plots them to graphs.









\--- Gazebo ---



my\_robot.urdf.xacro

The main robot model. It contains the ROS2 control block, the gz\_ros2\_control plugin and the glass ceiling.



display\_robot.launch.py

Starts gazebo and renders the robot.



controllers.yaml

Defines and activates the 2 controllers.



robot\_trajectory\_gazebo.m

Updated MATLAB robot simulation code with a ROS2 node. Pushes the joint positions over ROS2 joint position controllers and moves the two robot simulations together.



