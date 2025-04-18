"""
Description:
This script demonstrates how to generate and control a smooth, time-optimized
trajectory for a multi-degree-of-freedom (DoF) system using the Ruckig Online
Trajectory Generation (OTG) library. It sets up initial and target states
(positions, velocities, accelerations) for each axis, specifies kinematic limits
(max velocity, acceleration, jerk), and then iteratively computes the motion plan
in a control loop until the trajectory is complete.

At runtime, the code also shows how to interrupt the planned motion on‑the‑fly
(after 1 second), switching to an immediate stop trajectory with independent
axis deceleration by changing the control interface and disabling synchronization.
This ensures each DoF can brake as quickly as its limits allow.

Use Case:
Robotic manipulators, CNC machines, or automated camera rigs often require
precise, real‑time trajectory updates while respecting hardware kinematic
constraints. This example is particularly relevant when an unexpected event
(needs to stop immediately) occurs mid‑motion—each joint must decelerate
optimally without violating limits.

Functionality:
- **Ruckig**: instantiate the OTG object with a given number of DoFs and control cycle time.
- **InputParameter / OutputParameter**: define current and target states (pos, vel, acc).
- **Result enum**: check the status of trajectory generation (`Working`, `Finished`, etc.).
- **otg.update()**: compute the next trajectory segment each cycle.
- **ControlInterface**: switch between position and velocity control modes.
- **Synchronization**: enable or disable axis synchronization (`Synchronization.No` for independent stops).
- **Kinematic limits**: set per‑axis max velocity, acceleration, and jerk.
- **pass_to_input()**: feed the last output back into the next input for continuity.
- **On‑the‑fly interruption**: trigger a new stop trajectory at a specific runtime by updating targets and limits.
- **Performance metrics**: read out calculation duration and total trajectory duration.

"""

from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface, Synchronization


if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(3, 0.01)  # DoFs, control cycle
    inp = InputParameter(3)
    out = OutputParameter(3)

    # Initial state: position [m], velocity [m/s], acceleration [m/s²]
    inp.current_position     = [ 0.0,  0.0,  0.5]
    inp.current_velocity     = [ 0.0, -2.2, -0.5]
    inp.current_acceleration = [ 0.0,  2.5, -0.5]

    # Target state
    inp.target_position     = [ 5.0, -2.0, -3.5]
    inp.target_velocity     = [ 0.0, -0.5, -2.0]
    inp.target_acceleration = [ 0.0,  0.0,  0.5]

    # Kinematic limits
    inp.max_velocity     = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk         = [4.0, 3.0, 2.0]

    # Print header for logging
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

    # Generate the trajectory within the control loop
    first_output, out_list, time_offsets = None, [], []
    on_stop_trajectory = False
    res = Result.Working
    while res == Result.Working:
        # Compute next step
        res = otg.update(inp, out)

        # Log current time and positions
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))
        time_offsets.append(1.0 if on_stop_trajectory else 0.0)

        # After 1 second, trigger an immediate stop trajectory
        if out.time >= 1.0 and not on_stop_trajectory:
            print('Stop immediately!')
            on_stop_trajectory = True

            # Switch to velocity control, disable synchronization for independent stops
            inp.control_interface = ControlInterface.Velocity
            inp.synchronization    = Synchronization.No
            inp.target_velocity    = [0.0, 0.0, 0.0]
            inp.target_acceleration= [0.0, 0.0, 0.0]
            # Increase jerk limits for faster braking
            inp.max_jerk          = [12.0, 10.0,  8.0]

        # Feed back the previous output to maintain continuity
        out.pass_to_input(inp)

        if first_output is None:
            first_output = copy(out)

    # Print performance metrics
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

    # Plot the trajectory (uncomment if plotter is available)
    from pathlib import Path
    from plotter import Plotter
    
    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(
        project_path / 'examples' / '06_trajectory.pdf',
        otg, inp, out_list,
        plot_jerk=False,
        time_offsets=time_offsets
    )
