#!/usr/bin/env python3
"""
Description:
-------------------------------
Ruckig is an on‐the‐fly trajectory generator that computes smooth, jerk‐limited
S‑curve profiles for multi‑DoF systems in real time. By enforcing per‑axis limits
on velocity, acceleration and especially jerk, Ruckig produces motion commands
that are both responsive and gentle on mechanical hardware, avoiding the
vibrations and stress caused by abrupt changes.

Key ideas:
 1. **On‐the‐fly (OTG)**: At each control cycle, Ruckig computes just the next
    small segment of the trajectory, so there’s no long pre‐computation or
    blocking—perfect for real‐time loops.
 2. **Jerk‐limited S‐curves**: Ensures acceleration ramps smoothly, keeping
    actuator loads predictable and minimizing wear.
 3. **Multi‐DoF decoupling**: Treats each axis independently, yet synchronizes
    their durations to hit all targets simultaneously.

This example script shows how to:
  - Instantiate the Python bindings for Ruckig
  - Fill in current and target states plus limits
  - Step through the trajectory in a loop
  - Capture performance metrics (computation time and total trajectory duration)
  - (Optionally) hand off the results to an external plotter

Library Usage:
--------------
"""

from copy import copy
from ruckig import InputParameter, OutputParameter, Result, Ruckig

if __name__ == '__main__':
    # 1) Create the OTG instance and parameter containers
    otg = Ruckig(3, 0.01)           # 3 DoFs, 10 ms control cycle
    inp = InputParameter(3)
    out = OutputParameter(3)

    # 2) Define the current state
    inp.current_position     = [0.0,  0.0,  0.5]
    inp.current_velocity     = [0.0, -2.2, -0.5]
    inp.current_acceleration = [0.0,  2.5, -0.5]

    # 3) Define the target state
    inp.target_position      = [5.0, -2.0, -3.5]
    inp.target_velocity      = [0.0, -0.5, -2.0]
    inp.target_acceleration  = [0.0,  0.0,  0.5]

    # 4) Define per‐axis constraints
    inp.max_velocity     = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk         = [4.0, 3.0, 2.0]

    # 5) Header for console output
    print('\t'.join(['t'] + [f'pos{i}' for i in range(otg.degrees_of_freedom)]))

    # 6) Run the control loop
    first_output = None
    outputs = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        # Print time and positions
        print('\t'.join(
            [f'{out.time:0.3f}'] +
            [f'{p:0.3f}' for p in out.new_position]
        ))

        outputs.append(copy(out))
        out.pass_to_input(inp)

        if first_output is None:
            first_output = copy(out)

    # 7) Report performance
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} µs')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} s')

    # 8) (Optional) Plot the full trajectory
    from pathlib import Path
    from plotter import Plotter
    project_path = Path(__file__).parent.parent
    Plotter.plot_trajectory(
        project_path / 'examples' / 'trajectory.pdf',
        otg, inp, outputs, plot_jerk=False
    )
