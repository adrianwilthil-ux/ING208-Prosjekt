# -*- coding: utf-8 -*-
"""
Created on Wed Nov  4 08:01:48 2026

@author: ingeb
"""


import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3


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
    
q_lim = np.deg2rad([
    [-180, 180],
    [-90, 90],
    [-180, 180],
    [-180, 180],
    [-180, 180]
])

# -------------------------------------------------------
# Circle definition
# -------------------------------------------------------

radius = 0.05
center = np.array([0, 0, 1.5])

angle = np.deg2rad(0)

# Rotation of circle around Y
R = SE3.Ry(angle).R

# Circle normal (desired tool direction)
normal = R @ np.array([0, 0, 1])

# -------------------------------------------------------
# Generate points along circle
# -------------------------------------------------------

N = 1000

theta = np.linspace(0, 2*np.pi, N, endpoint=False)

# Circle in its local coordinate system
circle_local = np.array([
    radius * np.cos(theta),
    radius * np.sin(theta),
    np.zeros(N)
])

# Rotate and translate circle
circle_world = R @ circle_local + center.reshape(3, 1)




"""
# -------------------------------------------------------
# Line definition
# -------------------------------------------------------

length = 1
center = np.array([0, 0, 1.05])

angle = np.deg2rad(15)

# Rotation of circle around Y
R = SE3.Ry(angle).R

# Circle normal (desired tool direction)
normal = R @ np.array([0, 0, 1])

# -------------------------------------------------------
# Generate points along line
# -------------------------------------------------------

N = 1000

displacement = np.linspace(-length/2, length/2, N, endpoint=False)

# Circle in its local coordinate system
line_local = np.array([
    displacement,
    np.zeros(N),
    np.zeros(N)
])

# Rotate and translate circle
line_world = R @ line_local + center.reshape(3, 1)
"""
# -------------------------------------------------------
# Inverse kinematics
# -------------------------------------------------------

solutions = []
success = []
position_errors = []
orientation_errors = []
q_previous = np.zeros(robot.n)


for i in range(N):

    p = circle_world[:, i]
    T_des = SE3.Rt(R, p)

    q_sol, ok, pos_err, angle_err = ikine_drilling(
        robot,
        T_des,
        q_previous,

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


# -------------------------------------------------------
# Results
# -------------------------------------------------------

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

    print(f"Minimum angle: {np.nanmin(solutions[:, j]):.6f} rad "
          f"({np.rad2deg(np.nanmin(solutions[:, j])):.3f} deg)")

    print(f"Maximum angle: {np.nanmax(solutions[:, j]):.6f} rad "
          f"({np.rad2deg(np.nanmax(solutions[:, j])):.3f} deg)")

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
solutions=np.rad2deg(solutions)
print(solutions)
# -------------------------------------------------------
# Jacobian condition analysis
# -------------------------------------------------------
"""
for i in range(40, 61):

    if not success[i]:
        continue

    J = robot.jacob0(solutions[i])
    J = J[:5, :]

    s = np.linalg.svd(J, compute_uv=False)

    print(
        f"Index {i}: "
        f"Minimum singular value = {np.min(s):.8f}, "
        f"Condition number = {np.linalg.cond(J):.3f}"
    )
""" 
print("\nERROR STATISTICS")
print("-----------------------------")

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
"""
for i, link in enumerate(robot.links):
    print(i, link.name, link.isjoint)
# -------------------------------------------------------
# Forward kinematics
# -------------------------------------------------------
for i in range(robot.n + 1):

    T = robot.fkine(q, end=robot.links[i])

    print(f"\nFrame {i}:")
    print(T)
    
  
# Plot robot
q = np.zeros(robot.n)
# Zero velocity and acceleration
qd = np.zeros(robot.n)
qdd = np.zeros(robot.n)

# Required joint torques
tau = robot.rne(q, qd, qdd)

print("Joint torques [Nm]:")
print(tau)
#robot.plot(q, backend="pyplot", block=False)
"""