#!/usr/bin/env python3
"""
Description:
-----------------------
This example demonstrates how to plan and execute a smooth, time‑optimal motion trajectory
for a system with three degrees of freedom, passing through multiple intermediate waypoints
before reaching a final target state. It uses Ruckig, a real‑time online trajectory generation
library, to incrementally compute position, velocity, and acceleration setpoints at each
control cycle. By specifying maximum limits on velocity, acceleration, and jerk, the code
ensures physically feasible motions while allowing interruption if the computation takes
too long.

Library Use:
------------
- ruckig.InputParameter / OutputParameter: define the current state, constraints, and
  intermediate/final targets for each degree of freedom.
- ruckig.Ruckig: the core online trajectory generator that computes the next motion segment
  at each control step.
- copy.copy: to snapshot each output state for later analysis or plotting.
- (Optional) pathlib.Path and a custom Plotter for visualizing the resulting trajectory.

Use Case:
----------------------------
This pattern is typical in industrial robotics or automated machinery, where a robotic
arm or multi‑axis actuator must move smoothly through a series of workspace waypoints—
for example, welding along a predefined seam, painting complex surfaces, or pick‑and‑place
operations that require intermediate inspection or avoidance positions. By planning the
trajectory online, the controller can adapt in real time to new constraints or interrupts,
ensuring safe, efficient, and precise motion.
"""

from copy import copy

# Core Ruckig imports for online trajectory generation
from ruckig import InputParameter, OutputParameter, Result, Ruckig


if __name__ == '__main__':
    # Create the Ruckig OTG (Online Trajectory Generator) and parameter holders
    otg = Ruckig(3, 0.01, 10)           # 3 DoFs, 10 ms cycle, up to 10 intermediate waypoints
    inp = InputParameter(3)            # Input parameters for 3 DoFs
    out = OutputParameter(3, 10)       # Output parameters, matching DoFs and waypoint capacity

    # Initial state of the system
    inp.current_position     = [0.2,  0.0, -0.3]
    inp.current_velocity     = [0.0,  0.2,  0.0]
    inp.current_acceleration = [0.0,  0.6,  0.0]

    # Define intermediate waypoints to pass through
    inp.intermediate_positions = [
        [ 1.4, -1.6,  1.0],
        [-0.6, -0.5,  0.4],
        [-0.4, -0.35, 0.0],
        [ 0.8,  1.8, -0.1],
    ]

    # Final target state
    inp.target_position     = [0.5, 1.0, 0.0]
    inp.target_velocity     = [0.2, 0.0, 0.3]
    inp.target_acceleration = [0.0, 0.1, -0.1]

    # Motion constraints
    inp.max_velocity     = [1.0, 2.0, 1.0]
    inp.max_acceleration = [3.0, 2.0, 2.0]
    inp.max_jerk         = [6.0, 10.0, 20.0]

    # Maximum allowed calculation time per cycle (in microseconds)
    inp.interrupt_calculation_duration = 500

    # Print header for time and positions
    print('\t'.join(['t'] + [f"DoF{i}" for i in range(otg.degrees_of_freedom)]))

    # Control loop: update until the trajectory is complete
    out_list = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        if out.new_calculation:
            print('Updated the trajectory:')
            print(f'  Calculation duration: {out.calculation_duration:0.1f} [µs]')
            print(f'  Trajectory duration: {out.trajectory.duration:0.4f} [s]')

        # Log current time and positions
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))

        # Feed the computed setpoints back into the next update
        out.pass_to_input(inp)

    # Plot the full trajectory – requires a custom Plotter implementation
    from pathlib import Path
    from plotter import Plotter
    
    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(
        project_path / 'examples' / '04_trajectory.pdf',
        otg, inp, out_list, plot_jerk=False
    )
