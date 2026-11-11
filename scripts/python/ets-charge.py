# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 17:48:52 2026

@author: ingeb
"""


import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt

# Physical dimensions
L = np.array([0.15, 0.6, 0.3, 0.3, 0.152, 0.3])
M = np.array([1.0292436, 4.2266261, 1.942972,
              2.1265718, 3.6305687, 1.72])

# COM positions in each link frame
r1 = np.array([0, 0, 0.065114335])
r2 = np.array([L[1]/2, 0, 0])
r3 = np.array([1.5274698e-02, 0, 0])
r4 = np.array([0.1622270203, 0, 0])
r5 = np.array([-0.019119877, 0, 0])
rt = np.array([0.145, 0, 0])

# Inertia tensors about each link's COM
I1 = np.diag([0.0033650727, 0.0022757687, 0.0020656436])

I2 = np.array([
    [4.4263966e-03, 1.2423167e-07, 0.0000000e+00],
    [1.2423167e-07, 1.5722391e-01, 0.0000000e+00],
    [0.0000000e+00, 0.0000000e+00, 1.5717226e-01]
])

I3 = np.diag([3.1162346e-03, 6.1385077e-02, 6.0244306e-02])
I4 = np.diag([8.2938740e-03, 2.0093491e-02, 2.4317618e-02])
I5 = np.diag([6.1406326e-03, 1.9280078e-02, 1.9771259e-02])
It = np.diag([0.000194, 0.01390, 0.01390])


# -------------------------------------------------------
# Link inertial parameters
# -------------------------------------------------------

masses = np.array([ M[0], M[1], M[2], M[3], M[4], M[5]])

coms = [r1, r2, r3, r4, r5, rt]

inertias = [I1, I2, I3, I4, I5, It]

# -------------------------------------------------------
# Actuator and friction parameters
# -------------------------------------------------------

Jm = 1e-4       # Motor rotor inertia (kg m^2)
B = 0.01        # Viscous friction
Tc = np.array([0, 0])  # Coulomb friction [positive, negative]
G = 10          # Gear ratio


# -------------------------------------------------------
# Define ETS robot
# -------------------------------------------------------

ets = (
    rtb.ET.Rz() *
    rtb.ET.tz(L[0]) *
    rtb.ET.Rx(np.pi/2) *
    rtb.ET.Rz() *
    rtb.ET.tx(L[1]) *
    rtb.ET.Rz() *
    rtb.ET.tx(L[2]) *
    rtb.ET.Rx() *
    rtb.ET.tx(L[3]) *
    rtb.ET.Rz() *
    rtb.ET.tx(L[4]) *
    rtb.ET.tx(L[5]) 
    )

robot = rtb.ERobot(ets, name="Drilling Robot")
# Define TCP frame
robot.tool = SE3.Ry(np.pi/2).A
# Gravity vector [m/s^2]
robot.gravity = np.array([0, 0, -9.81])

# Display robot
print(robot)
q = np.zeros(robot.n)
# -------------------------------------------------------
# Assign dynamics to robot links
# -------------------------------------------------------

for i, link in enumerate(robot.links):

    link.m = masses[i]
    link.r = coms[i]
    link.I = inertias[i]

moving_links = [link for link in robot.links if link.isjoint]

for link in moving_links:
    link.Jm = Jm
    link.B = B
    link.Tc = Tc
    link.G = G


q = np.zeros(robot.n)
# Zero velocity and acceleration
qd = np.zeros(robot.n)
qdd = np.zeros(robot.n)

# Three joint configurations (radians)
q1 = np.array([0,  0,  -np.pi/2, 0,  0])#Tool change position
q2 = np.array([0,  0,  np.pi/2, 0,  0])#Rest position

# Segment durations
T = 4.0


# Time step
dt = 0.01

t= np.arange(0, T + dt, dt)


# Generate trajectories
traj = rtb.jtraj(q1, q2, t)

# Extract trajectory variables
q = traj.q
qd = traj.qd
qdd = traj.qdd

tau = robot.rne(q, qd, qdd)
fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)


for i in range(robot.n):
    ax[0].plot(t, qd[:, i], label=f'Joint {i+1}')
    ax[1].plot(t, qdd[:, i], label=f'Joint {i+1}')
    ax[2].plot(t, tau[:, i], label=f'Joint {i+1}')

ax[0].set_ylabel('Velocity (rad/s)')
ax[1].set_ylabel('Acceleration (rad/s²)')
ax[2].set_ylabel('Torque (Nm)')
ax[2].set_xlabel('Time (s)')

for a in ax:
    a.grid(True)
    a.legend()

plt.tight_layout()
plt.show()

max_velocity = np.max(np.abs(qd), axis=0)
max_acceleration = np.max(np.abs(qdd), axis=0)
max_torque = np.max(np.abs(tau), axis=0)

for i in range(robot.n):
    print(f"\nJoint {i+1}")
    print(f"Max velocity:     {max_velocity[i]:.4f} rad/s")
    print(f"Max acceleration: {max_acceleration[i]:.4f} rad/s²")
    print(f"Max torque:       {max_torque[i]:.4f} Nm")