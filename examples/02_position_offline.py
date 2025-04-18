#!/usr/bin/env python3
"""
Description:
-----------------------
This script demonstrates an **offline** S‑curve trajectory planner for a 3‑DoF system
(e.g., a robotic arm end‑effector or a 3‑axis positioning stage). It takes the current
Cartesian state (position, velocity, acceleration) and smoothly transitions to a target
state, obeying per‑axis limits on velocity, acceleration and jerk. Unlike a real‑time
control loop, this offline approach computes the entire profile at once—ideal for
generating open‑loop motion plans, simulation, or pre‑computing setpoint tables.

Use Case:
-------------------
Imagine a pick‑and‑place robot that must move its gripper from one 3D point to another
without causing mechanical shock. Before execution, you generate the full trajectory,
inspect key points (duration, extrema), and then feed it to your low‑level controller
or motion playback system.

Library Usage:
--------------
- `InputParameter(3)`: holds start/goal states and constraints for 3 axes.
- `Ruckig(3)`: offline trajectory generator (no control cycle needed).
- `Trajectory(3)`: container for the full motion profile.
- `Result`: status of the generation call.
"""

from ruckig import InputParameter, Ruckig, Trajectory, Result

if __name__ == '__main__':
    # 1) Define current state of the 3 axes (e.g., X, Y, Z)
    inp = InputParameter(3)
    inp.current_position     = [0.0,  0.0,  0.5]
    inp.current_velocity     = [0.0, -2.2, -0.5]
    inp.current_acceleration = [0.0,  2.5, -0.5]

    # 2) Define target state
    inp.target_position     = [ 5.0, -2.0, -3.5]
    inp.target_velocity     = [ 0.0, -0.5, -2.0]
    inp.target_acceleration = [ 0.0,  0.0,  0.5]

    # 3) Set symmetric max limits
    inp.max_velocity     = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk         = [4.0, 3.0, 2.0]

    # 4) (Optional) Different limits in the negative direction
    inp.min_velocity     = [-1.0,  -0.5, -3.0]
    inp.min_acceleration = [-2.0,  -1.0, -2.0]

    # 5) Create generator and trajectory container
    otg        = Ruckig(3)              # offline: no cycle time needed
    trajectory = Trajectory(3)

    # 6) Compute the full trajectory
    result = otg.calculate(inp, trajectory)
    if result == Result.ErrorInvalidInput:
        raise RuntimeError("Invalid input parameters for trajectory generation")

    print(f"→ Trajectory duration: {trajectory.duration:.4f} s")

    # 7) Query state at an arbitrary time (e.g., t = 1.0 s)
    t_query = 1.0
    pos, vel, acc = trajectory.at_time(t_query)
    print(f"→ State at t={t_query:.4f} s: pos={pos}, vel={vel}, acc={acc}")

    # 8) Inspect position extrema over the entire motion
    print(f"→ Position extrema per axis: {trajectory.position_extrema}")

