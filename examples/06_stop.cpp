/**
 * Description:
 *
 * This program showcases real‐time, online trajectory generation for a
 * 3‑degree‑of‑freedom (3‑DoF) system using the Ruckig library. It performs
 * the following high‑level steps:
 *   1. Initializes an online trajectory generator (OTG) with a 10 ms control cycle.
 *   2. Defines the current state (position, velocity, acceleration) and desired
 *      target state for each of the three axes.
 *   3. Specifies per‑axis dynamic constraints: maximum velocity, acceleration,
 *      and jerk.
 *   4. Enters a control loop where, at each cycle:
 *      • `update()` computes the next trajectory segment.
 *      • The new positions are printed alongside the current time stamp.
 *      • After 1.0 s, the code triggers an immediate stop:
 *         – Switches to velocity control to focus on deceleration.
 *         – Disables synchronization so each axis stops independently and
 *           as fast as its own limits allow.
 *         – Sets target velocities and accelerations to zero.
 *         – Raises jerk limits to brake more aggressively.
 *      • Feeds the previous output back into the next input via
 *        `pass_to_input()` for continuous online recalculation.
 *   5. Continues until the trajectory is complete, then reports the total
 *      duration of the stop trajectory.
 *
 * Use Cases:
 *   • **Industrial Robotics**: Smooth and safe motion planning for robotic
 *     manipulators or CNC machines, respecting strict kinematic limits.
 *   • **Automated Vehicles**: Generating online speed profiles under dynamic
 *     constraints for braking maneuvers or path tracking.
 *   • **Motion Simulation & Testing**: Evaluating control algorithms with
 *     on‑the‑fly trajectory adjustments and real‑time logging.
 *
 * Features:
 *   • **Ruckig<3> otg(0.01)**: Creates a 3‑DoF OTG with a 10 ms control interval.
 *   • **InputParameter/OutputParameter**: Data structures for feeding state
 *     and retrieving trajectory results.
 *   • **update()**: Performs incremental trajectory calculation, returning
 *     `Result::Working` until completion.
 *   • **pass_to_input()**: Chains the output back into the input for the
 *     next cycle, enabling continuous online updates.
 *   • **ControlInterface::Velocity**: Switches the control mode to velocity
 *     control for immediate stopping.
 *   • **Synchronization::None**: Disables axis synchronization so each DoF
 *     decelerates independently.
 *   • **Dynamic Reconfiguration**: Adjusts target states and kinematic limits
 *     (e.g., jerk) at runtime to adapt the trajectory mid‑flight.
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3> otg(0.01);
    InputParameter<3> input;
    OutputParameter<3> output;

    // Set input parameters
    input.current_position     = { 0.0,  0.0,  0.5 };
    input.current_velocity     = { 0.0, -2.2, -0.5 };
    input.current_acceleration = { 0.0,  2.5, -0.5 };

    input.target_position      = { 5.0, -2.0, -3.5 };
    input.target_velocity      = { 0.0, -0.5, -2.0 };
    input.target_acceleration  = { 0.0,  0.0,  0.5 };

    input.max_velocity         = { 3.0,  1.0,  3.0 };
    input.max_acceleration     = { 3.0,  2.0,  1.0 };
    input.max_jerk             = { 4.0,  3.0,  2.0 };

    // Generate the trajectory within the control loop
    std::cout << "t | position" << std::endl;
    bool on_stop_trajectory = false;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        // Activate stop trajectory after 1s
        if (output.time >= 1.0 && !on_stop_trajectory) {
            std::cout << "Stop immediately." << std::endl;
            on_stop_trajectory = true;

            // Switch to velocity control and disable synchronization so each
            // DoF stops as fast as possible independently
            input.control_interface    = ControlInterface::Velocity;
            input.synchronization      = Synchronization::None;
            input.target_velocity      = { 0.0,  0.0,  0.0 };
            input.target_acceleration  = { 0.0,  0.0,  0.0 };
            input.max_jerk             = { 12.0, 10.0,  8.0 };
        }

        // Feed the last output back into the next input
        output.pass_to_input(input);
    }

    std::cout << "Stop trajectory duration: "
              << output.trajectory.get_duration()
              << " [s]." << std::endl;
    return 0;
}
