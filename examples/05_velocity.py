"""
Description:
This script showcases the use of the Ruckig Online Trajectory 
Generation (OTG) library to compute real-time, time-optimal 
motion trajectories for a multi-degree-of-freedom (DoF) system. 
By iteratively updating kinematic states within a control loop, 
Ruckig ensures that position, velocity, acceleration, and jerk 
constraints are respected at every step, producing smooth and 
feasible motion profiles.

Use Case
-------------------
This pattern is typical in robotics (e.g., robot arms, CNC machines, pick-and-place systems),
automated camera gimbals, or any application requiring smooth, time‑optimal motion
between states under strict kinematic limits. By running the Ruckig OTG (Online Trajectory
Generation) inside a control loop, you can continuously update commands to a motor driver
or controller, ensuring adherence to acceleration and jerk constraints for safe, precise motion.

Key Features
-----------------
  • Degrees of Freedom (DoFs): 3 independent motion axes  
  • Control Cycle: 10 ms update interval (0.01 s)  
  • Control Interface: Velocity-based control (specifying target velocities and accelerations)  
  • State Specification:  
      – current_position, current_velocity, current_acceleration  
      – target_velocity, target_acceleration  
  • Kinematic Constraints:  
      – max_acceleration per axis  
      – max_jerk per axis  
  • Ruckig API Calls:  
      – `Ruckig(...)` to create an OTG instance  
      – `InputParameter` / `OutputParameter` for passing states  
      – `otg.update(...)` to compute next trajectory segment  
      – `out.pass_to_input(inp)` to feed the output back as the next input  
  • Performance Metrics:  
      – `calculation_duration` for per-step compute time  
      – `trajectory.duration` for total motion time  
  • Data Collection: accumulating `out_list` for post‑processing or plotting  
  • Optional Plotting: using an external Plotter to save a PDF of the full trajectory  

Functionality
------------------
- Creation of a 3‑DoF OTG with 10 ms control cycle  
- Velocity‑control interface selection  
- Definition of current and target kinematic states  
- Enforcement of per‑axis acceleration and jerk limits  
- Iterative update loop generating discrete trajectory points  
- Collection of output samples for downstream visualization  
- Reporting of compute time and trajectory duration  
"""

from copy import copy
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface

if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(3, 0.01)  # DoFs, control cycle
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.control_interface = ControlInterface.Velocity

    inp.current_position = [0.0, 0.0, 0.5]
    inp.current_velocity = [3.0, -2.2, -0.5]
    inp.current_acceleration = [0.0, 2.5, -0.5]

    inp.target_velocity = [0.0, -0.5, -1.5]
    inp.target_acceleration = [0.0, 0.0, 0.5]

    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [6.0, 6.0, 4.0]

    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))

        out.pass_to_input(inp)

        if not first_output:
            first_output = copy(out)

    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

    # Plot the trajectory
    from pathlib import Path
    from plotter import Plotter

    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(project_path / 'examples' / '05_trajectory.pdf', otg, inp, out_list, plot_jerk=False)
