"""
This example demonstrates how to generate a multi-segment, 
time-optimal trajectory for a system with three degrees of freedom 
(DoFs) using Ruckig’s “intermediate waypoints” feature. 
By specifying a sequence of waypoints between the current and 
target states, the motion planner computes a smooth trajectory 
that passes through each waypoint in order, respecting kinematic 
constraints and allowing for minimum durations per segment.

the script does the following:
1. **Initialization**  
   - Creates a Ruckig OTG (online trajectory generator) instance 
     for 3 DoFs, with a 10 ms control cycle and memory allocated 
     for up to 10 intermediate waypoints.
   - Prepares InputParameter and OutputParameter objects matching 
     the DoFs and waypoint capacity.

2. **State and Waypoint Definition**  
   - Sets the **current** state (position, velocity, acceleration) 
     of the system.
   - Defines a list of **intermediate positions** the trajectory must 
     visit in sequence.
   - Specifies the **target** end state (position, velocity, 
     acceleration).
   - Applies per-DoF limits on maximum velocity, acceleration, and jerk.
   - Assigns a **minimum duration** for each segment (i.e., between 
     each pair of consecutive waypoints, including start and end), 
     ensuring some segments cannot be shorter than a specified time.

3. **Real-Time Control Loop**  
   - Enters a loop calling `otg.update(inp, out)` at each control cycle.
   - Records each new output position and automatically feeds the 
     previous output back into the input for the next cycle.
   - Continues until the result code indicates the trajectory is 
     complete (i.e., `Result.Working` turns into `Result.Finished` or an error).

4. **Performance Metrics**  
   - Captures the computation time of the first update call.
   - Reports the total duration of the planned trajectory once complete.

5. **Visualization**  
   - Includes commented-out code to plot the resulting trajectory 
     using an external plotting utility.

**Use case:**  
This pattern is typical for robotic manipulators, CNC machines, 
or automated vehicles that must navigate through predefined 
checkpoints smoothly and as quickly as possible without violating 
kinematic constraints. For instance, a 3‑axis robotic arm in an 
assembly line might need to move through several tool‑change 
positions while respecting speed, acceleration, and jerk limits 
to avoid mechanical stress or payload oscillations.

**Features:**  
- **Intermediate waypoints** (`inp.intermediate_positions`)  
- **Per‑section minimum durations** (`inp.per_section_minimum_duration`)  
- **Per‑DoF kinematic limits** (`max_velocity`, `max_acceleration`, `max_jerk`)  
- **Online trajectory generation loop** (`otg.update()` + `out.pass_to_input()`)  
- **Performance measurement** (`calculation_duration`, `trajectory.duration`)
"""

from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig


if __name__ == '__main__':
    # Create Ruckig OTG instance and input/output parameter objects
    otg = Ruckig(3, 0.01, 10)  # 3 DoFs, 10 ms control cycle, room for 10 waypoints
    inp = InputParameter(3)
    out = OutputParameter(3, 10)

    # Current state
    inp.current_position     = [0.8,  0.0,  0.5]
    inp.current_velocity     = [0.0,  0.0,  0.0]
    inp.current_acceleration = [0.0,  0.0,  0.0]

    # Intermediate waypoints the trajectory must pass through
    inp.intermediate_positions = [
        [ 1.4, -1.6,  1.0],
        [-0.6, -0.5,  0.4],
        [-0.4, -0.35, 0.0],
        [-0.2,  0.35, -0.1],
        [ 0.2,  0.5,  -0.1],
        [ 0.8,  1.8,  -0.1],
    ]

    # Target end state
    inp.target_position     = [0.5,  1.2, 0.0]
    inp.target_velocity     = [0.0,  0.0, 0.0]
    inp.target_acceleration = [0.0,  0.0, 0.0]

    # Kinematic limits per DoF
    inp.max_velocity     = [3.0, 2.0, 2.0]
    inp.max_acceleration = [6.0, 4.0, 4.0]
    inp.max_jerk         = [16.0, 10.0, 20.0]

    # Minimum allowed duration for each trajectory segment
    # (number of waypoints + 1 segments; zeros mean no minimum)
    inp.per_section_minimum_duration = [0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0]

    # Print header for time vs. position output
    print('\t'.join(['t'] + [f'axis_{i}' for i in range(otg.degrees_of_freedom)]))

    # Run the control loop until the trajectory is complete
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))
        out.pass_to_input(inp)
        if first_output is None:
            first_output = copy(out)

    # Report performance
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} µs')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} s')

    # Plot the trajectory to a PDF (requires external Plotter utility)
    from pathlib import Path
    from plotter import Plotter
    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(
        project_path / 'examples' / '08_trajectory.pdf',
        otg, inp, out_list,
        plot_jerk=False
    )
