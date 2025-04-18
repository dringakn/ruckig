#!/usr/bin/env python3
"""
Description:
This script demonstrates how to perform online, real-time trajectory generation (OTG) 
for a multi‑degree‑of‑freedom (DOF) system using the Ruckig library. It initializes 
an OTG object for a 3‑DOF task with a given control cycle (0.01 s), sets up the current 
and target states (position, velocity, acceleration) as well as dynamic limits 
(maximum velocity, acceleration, jerk), and enforces a minimum trajectory duration. 
Within a control loop, it repeatedly computes the next step of the trajectory until 
the motion is complete, printing out the time-stamped positions at each update. 
Finally, it reports performance metrics (calculation duration and total trajectory 
duration) and (optionally) plots the resulting trajectory.

Use Case:
This pattern is common in robotics, CNC machining, or any automated motion control 
scenario where smooth, jerk‑limited trajectories must be computed on-the-fly. 
For example, a 3‑axis robotic arm moving from one point to another while respecting 
velocity, acceleration, and jerk constraints to avoid mechanical stress and ensure 
precise timing in pick‑and‑place operations.

Features and Functionality:
- **Ruckig( degrees_of_freedom, control_cycle )**  
  Creates an online trajectory generator for a system with the given number of axes 
  and control update interval.
- **InputParameter( degrees_of_freedom ) / OutputParameter( degrees_of_freedom )**  
  Data containers specifying the current and target states, limits, and capturing the 
  result of each update step.
- **State Specification:**  
  - `current_position`, `current_velocity`, `current_acceleration`  
  - `target_position`, `target_velocity`, `target_acceleration`  
- **Dynamic Limits:**  
  - `max_velocity`, `max_acceleration`, `max_jerk`  
- **Minimum Duration:**  
  - Forces trajectories to last at least the given time, useful when decelerating to zero.
- **Control Loop with `otg.update(...)`:**  
  Iteratively computes the next trajectory segment until `Result` is no longer `Working`.
- **Performance Metrics:**  
  - `calculation_duration` measures compute time per update.  
  - `trajectory.duration` gives total planned motion time.
- **State Feedback:**  
  - `out.pass_to_input(inp)` feeds the newly computed state back as the next input.
- **Copying Outputs:**  
  - Uses `copy()` to snapshot each `OutputParameter` instance for later analysis or plotting.

"""

from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig


if __name__ == '__main__':
    # Initialize online trajectory generator for 3 DOFs with 10 ms control cycle
    otg = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    # Set current state (position [m], velocity [m/s], acceleration [m/s²])
    inp.current_position     = [ 0.0,  0.0,  0.5]
    inp.current_velocity     = [ 0.0, -2.2, -0.5]
    inp.current_acceleration = [ 0.0,  2.5, -0.5]

    # Set target state
    inp.target_position      = [-5.0, -2.0, -3.5]
    inp.target_velocity      = [ 0.0, -0.5, -2.0]
    inp.target_acceleration  = [ 0.0,  0.0,  0.5]

    # Define dynamic limits
    inp.max_velocity     = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk         = [4.0, 3.0, 2.0]

    # Enforce a minimum trajectory duration (seconds)
    inp.minimum_duration = 5.0

    # Header for console output
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

    # Run control loop: generate trajectory until completion
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        # Print time and new positions for each axis
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))

        # Feed back the result as next input
        out.pass_to_input(inp)

        # Capture the very first output for timing metrics
        if not first_output:
            first_output = copy(out)

    # Display performance metrics
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

    # Plot the trajectory to a PDF file
    from pathlib import Path
    from plotter import Plotter
    
    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(
        project_path / 'examples' / '07_trajectory.pdf',
        otg, inp, out_list, plot_jerk=False
    )
