# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 17:48:52 2026

@author: ingeb
"""


import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt

#inverse kinematics solver
def ikine_drilling(robot, T_des, q0,
                   max_iter=200,
                   tol_pos=1e-5,
                   tol_angle=np.deg2rad(0.1),
                   damping=1e-3):

    q = np.array(q0, dtype=float).copy()

    # Characteristic length for balancing position and orientation
    ell = 0.2  # metres

    for iteration in range(max_iter):

        # Actual TCP pose
        T = robot.fkine(q)

        p = T.t
        R = T.R

        # Position error
        ep = T_des.t - p
        pos_error = np.linalg.norm(ep)

        # Actual and desired TCP Z axes
        z = R[:, 2]
        zd = T_des.R[:, 2]

        # Angle between the two axes
        dot = np.clip(np.dot(z, zd), -1.0, 1.0)
        theta = np.arccos(dot)

        # Check convergence
        if pos_error < tol_pos and theta < tol_angle:
            return q, True, pos_error, theta

        # Rotation vector that aligns actual Z with desired Z
        cross = np.cross(z, zd)
        cross_norm = np.linalg.norm(cross)

        if cross_norm < 1e-10:

            if dot < 0:
                # Antiparallel axes: rotation direction is ambiguous
                return q, False, pos_error, theta

            rot_error = np.zeros(3)

        else:
            rot_error = theta * cross / cross_norm

        # Construct two orthogonal directions perpendicular to z
        reference = np.array([1.0, 0.0, 0.0])

        if abs(np.dot(reference, z)) > 0.9:
            reference = np.array([0.0, 1.0, 0.0])

        e1 = np.cross(z, reference)
        e1 /= np.linalg.norm(e1)

        e2 = np.cross(z, e1)

        E = np.column_stack((e1, e2))

        # Robot spatial Jacobian at TCP
        J = robot.jacob0(q)

        Jv = J[:3, :]
        Jw = J[3:, :]

        # Five-dimensional task Jacobian
        J_task = np.vstack((
            Jv,
            ell * E.T @ Jw
        ))

        # Desired task-space correction
        error = np.concatenate((
            ep,
            ell * E.T @ rot_error
        ))

        # Damped least-squares solution
        dq = J_task.T @ np.linalg.solve(
            J_task @ J_task.T
            + damping**2 * np.eye(5),
            error
        )

        # Limit individual iteration step
        max_step = 0.2
        step_norm = np.linalg.norm(dq)

        if step_norm > max_step:
            dq *= max_step / step_norm

        q += dq

    # Final error check
    T = robot.fkine(q)

    ep = T_des.t - T.t

    z = T.R[:, 2]
    zd = T_des.R[:, 2]

    theta = np.arccos(
        np.clip(np.dot(z, zd), -1.0, 1.0)
    )

    return q, False, np.linalg.norm(ep), theta
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

#Tilt angle
angle = np.deg2rad(15)

# Gravity vector [m/s^2]
g_world = np.array([0, 0, -9.81])

robot.gravity = SE3.Ry(angle).R @ g_world

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


# Segment durations
T = 200.0
dt=0.1



# -------------------------------------------------------
# Line definition
# -------------------------------------------------------

length = 0.2
offset = np.array([0, 0.5, 0.95])


# Rotation of drilling line around Y
R = SE3.Ry(angle).R

# Drilling direction
normal = R @ np.array([0, 0, 1])

# -------------------------------------------------------
# Generate points along tilted line
# -------------------------------------------------------

N = 2000

displacement = np.linspace(0, length, N, endpoint=False)

# Line in local coordinates
line_local = np.array([
    np.zeros(N),
    np.zeros(N),
    displacement
])

# Rotate line into base coordinates and add offset
line = R @ line_local + offset.reshape(3, 1)
# -------------------------------------------------------
# Time vector
# -------------------------------------------------------


t = np.linspace(0, T, N, endpoint=False)

# -------------------------------------------------------
# Inverse kinematics
# -------------------------------------------------------

solutions = []
success = []
position_errors = []
orientation_errors = []
q_previous = np.zeros(robot.n)


for i in range(N):

    p = line[:, i]
    T_des = SE3.Rt(R, p)

    q_sol, ok, pos_err, angle_err = ikine_drilling(
        robot,
        T_des,
        q_previous
    )

    if ok:

        q_previous = q_sol

        solutions.append(q_sol)
        success.append(True)

        position_errors.append(pos_err)
        orientation_errors.append(angle_err)

    else:

        solutions.append(np.full(robot.n, np.nan))
        success.append(False)

        position_errors.append(pos_err)
        orientation_errors.append(angle_err)

solutions = np.array(solutions)
success = np.array(success)

position_errors = np.array(position_errors)
orientation_errors = np.array(orientation_errors)

print("\nIK RESULTS")
print("-----------------------------")
print("Reachable points:", np.sum(success), "/", N)
print("Reachability:", 100*np.mean(success), "%")
# Joint movement between consecutive points [rad]
dq = np.diff(solutions, axis=0)

# Account for angular wrapping at +/- pi
dq = (dq + np.pi) % (2 * np.pi) - np.pi

# Absolute joint movements
dq_abs = np.abs(dq)

print("\nJOINT CONTINUITY")
print("-----------------------------")

for j in range(robot.n):
    print(f"\nJoint {j+1}:")
    print(f"Maximum delta: {np.max(dq_abs[:, j]):.6f} rad "
          f"({np.rad2deg(np.max(dq_abs[:, j])):.3f} deg)")
    print(f"Average delta: {np.mean(dq_abs[:, j]):.6f} rad "
          f"({np.rad2deg(np.mean(dq_abs[:, j])):.3f} deg)")

print("\nOverall:")
print("Maximum joint delta:",
      np.rad2deg(np.max(dq_abs)), "deg")
print("Average joint delta:",
      np.rad2deg(np.mean(dq_abs)), "deg")
if not np.all(success):
    failed = np.where(~success)[0]
    print("Failed point indices:", failed)

print(
    "Maximum position error:",
    np.nanmax(position_errors) * 1000,
    "mm"
)

print(
    "Mean position error:",
    np.nanmean(position_errors) * 1000,
    "mm"
)

print(
    "Maximum orientation error:",
    np.rad2deg(np.nanmax(orientation_errors)),
    "degrees"
)

print(
    "Mean orientation error:",
    np.rad2deg(np.nanmean(orientation_errors)),
    "degrees"
)


q_traj = solutions
qd_traj = np.gradient(q_traj, dt, axis=0)
qdd_traj = np.gradient(qd_traj, dt, axis=0)


# External force in TCP frame [N]
F_tcp = np.array([0, 0, 100])

# External moment in TCP frame [Nm]
M_tcp = np.array([0, 0, 5])

# External wrench
wrench_tcp = np.concatenate((F_tcp, M_tcp))

# Initialize torque arrays

tau_no_force = np.zeros((len(q_traj), robot.n))
tau_with_force = np.zeros((len(q_traj), robot.n))

#Torque calculation
for i in range(len(q_traj)):

    # TCP pose
    T_tcp = robot.fkine(q_traj[i])

    # Transform external wrench from TCP to base frame
    F_base = T_tcp.R @ F_tcp
    M_base = T_tcp.R @ M_tcp

    wrench_base = np.concatenate((F_base, M_base))

    # Inverse dynamics without external forces
    tau_no_force[i, :] = robot.rne(
        q_traj[i],
        qd_traj[i],
        qdd_traj[i]
    )

    # Jacobian at TCP
    J = robot.jacob0(q_traj[i])

    # External joint torques
    tau_ext = J.T @ wrench_base

    # Total joint torques
    tau_with_force[i, :] = tau_no_force[i, :] + tau_ext  
  



fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)


for i in range(robot.n):
    ax[0].plot(t, qd_traj[:, i], label=f'Joint {i+1}')
    ax[1].plot(t, qdd_traj[:, i], label=f'Joint {i+1}')
    ax[2].plot(t, tau_with_force[:, i], label=f'Joint {i+1}')

ax[0].set_ylabel('Velocity (rad/s)')
ax[1].set_ylabel('Acceleration (rad/s²)')
ax[2].set_ylabel('Torque (Nm)')
ax[2].set_xlabel('Time (s)')

for a in ax:
    a.grid(True)
    a.legend()

plt.tight_layout()
plt.show()


max_velocity = np.max(np.abs(qd_traj), axis=0)
max_acceleration = np.max(np.abs(qdd_traj), axis=0)
max_torque = np.max(np.abs(tau_with_force), axis=0)
max_unloaded = np.max(np.abs(tau_no_force), axis=0)
min_position = np.min(q_traj, axis=0)
max_position = np.max(q_traj, axis=0)
average_torque = np.nanmean(np.abs(tau_with_force), axis=0)

for i in range(robot.n):
    print(f"\nJoint {i+1}")
    print(f"Min position: {min_position[i]:.4f} rad "
      f"({np.rad2deg(min_position[i]):.2f} deg)")
    print(f"Max position: {max_position[i]:.4f} rad "
      f"({np.rad2deg(max_position[i]):.2f} deg)")
    print(f"Max velocity:     {max_velocity[i]:.4f} rad/s")
    print(f"Max acceleration: {max_acceleration[i]:.4f} rad/s²")
    print(f"Max torque:       {max_torque[i]:.4f} Nm")
    print(f"Average torque:  {average_torque[i]:.4f} Nm")
    print(f"Max unloaded:       {max_unloaded[i]:.4f} Nm")