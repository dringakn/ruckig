#!/usr/bin/env python3
"""
Description:
-----------------------
In many industrial and service‑robot applications, you need to move a multi‑axis manipulator
smoothly through a sequence of key poses—think welding along a seam, spray‑painting a curved surface,
or pick‑and‑place operations that must avoid obstacles. This example shows how to use Ruckig’s
online trajectory generator (OTG) with **intermediate waypoints** to compute a single, jerk‑limited
S‑curve path through a user‑defined list of via points, all within a real‑time control loop.

Use Case:
Suppose you have a 3‑DoF robot arm inspecting multiple inspection points on a turbine blade:
you’d program the blade landmarks as `intermediate_positions`, then Ruckig will seamlessly
blend from your start pose, through each waypoint, to your final target, all while respecting
velocity, acceleration, and jerk limits to protect your hardware and ensure smooth motion.

Library Usage:
--------------
- **Ruckig(3, dt, N_waypoints)**: instantiate a 3‑DoF OTG with 10 ms cycle and buffer for 10 via points  
- **InputParameter** / **OutputParameter**: hold your current/target states plus intermediate positions  
- **update()**: called each control tick to produce the next sample until the trajectory is complete  
"""

from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig

if __name__ == '__main__':
    # 1) Instantiate Ruckig: 3 axes, 0.01 s cycle, room for 10 intermediate waypoints
    otg = Ruckig(3, 0.01, 10)
    inp = InputParameter(3)
    out = OutputParameter(3, 10)

    # 2) Define your start state
    inp.current_position     = [0.2,  0.0, -0.3]
    inp.current_velocity     = [0.0,  0.2,  0.0]
    inp.current_acceleration = [0.0,  0.6,  0.0]

    # 3) List of intermediate waypoints (e.g. inspection or spray points)
    inp.intermediate_positions = [
        [ 1.4, -1.6,  1.0],
        [-0.6, -0.5,  0.4],
        [-0.4, -0.35, 0.0],
        [ 0.8,  1.8, -0.1],
    ]

    # 4) Final target state
    inp.target_position     = [0.5, 1.0, 0.0]
    inp.target_velocity     = [0.2, 0.0, 0.3]
    inp.target_acceleration = [0.0, 0.1,-0.1]

    # 5) Motion limits per axis
    inp.max_velocity     = [1.0, 2.0, 1.0]
    inp.max_acceleration = [3.0, 2.0, 2.0]
    inp.max_jerk         = [6.0,10.0,20.0]

    # Header for console output
    print('\t'.join(['t'] + [f'axis{i}' for i in range(otg.degrees_of_freedom)]))

    # 6) Control loop: call update() until trajectory done
    first_out, history = None, []
    status = Result.Working
    while status == Result.Working:
        status = otg.update(inp, out)
        # Print current time and positions
        print('\t'.join([f'{out.time:0.3f}'] +
                        [f'{p:0.3f}' for p in out.new_position]))
        history.append(copy(out))

        # For performance metrics
        if first_out is None:
            first_out = copy(out)

        # Prepare for next cycle
        out.pass_to_input(inp)

    # 7) Report timings
    print(f'Calculation duration: {first_out.calculation_duration:0.1f} µs')
    print(f'Trajectory duration:   {first_out.trajectory.duration:0.4f} s')

    # 8) Plotting with your preferred tool:
    from plotter import Plotter
    Plotter.plot_trajectory('trajectory.pdf', otg, inp, history, plot_jerk=False)
