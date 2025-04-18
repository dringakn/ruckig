/**
 * Description:
 * --------------------------------
 * This simple program demonstrates on‑the‑fly, real‑time trajectory generation
 * for a 3‑DoF system using Ruckig’s S‑curve (jerk‑limited) profiles. It:
 *
 * 1. **Initializes** a Ruckig Online Trajectory Generator (OTG) with a 10 ms control interval.
 * 2. **Defines** the system’s current state (position, velocity, acceleration) and the desired
 *    end state, along with per‑axis maximums on velocity, acceleration, and jerk.
 * 3. **Runs** a loop where each call to `otg.update(...)` computes the next slice (10 ms) of the
 *    trajectory, guaranteeing that no limits are violated and that motion is smooth (no sudden
 *    changes in acceleration).
 * 4. **Outputs** the time and new positions at each step, then feeds that output back into the
 *    input for the next iteration, closing the real‑time control loop.
 * 5. **Reports** the total trajectory duration once the target state is reached.
 *
 * Ruckig’s design makes it trivial to swap in new targets or dynamic limits mid‑flight,
 * ensuring actuator‑friendly motion without offline preprocessing.
 */
#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // 1) Create the trajectory generator (3 axes, 10 ms control cycle)
    Ruckig<3> otg(0.01);
    InputParameter<3> input;
    OutputParameter<3> output;

    // 2) Define current state
    input.current_position     = { 0.0,  0.0,  0.5 };
    input.current_velocity     = { 0.0, -2.2, -0.5 };
    input.current_acceleration = { 0.0,  2.5, -0.5 };

    // 3) Define target state
    input.target_position      = { 5.0, -2.0, -3.5 };
    input.target_velocity      = { 0.0, -0.5, -2.0 };
    input.target_acceleration  = { 0.0,  0.0,  0.5 };

    // 4) Set per‑axis limits
    input.max_velocity     = { 3.0, 1.0, 3.0 };
    input.max_acceleration = { 3.0, 2.0, 1.0 };
    input.max_jerk         = { 4.0, 3.0, 2.0 };

    // 5) Run the control loop and print time | position
    std::cout << "t [s] | position [x, y, z]" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout
            << output.time << " | "
            << join(output.new_position)
            << std::endl;

        // feed the new state back for the next iteration
        output.pass_to_input(input);
    }

    // 6) Finished: print total trajectory duration
    std::cout
        << "Trajectory duration: "
        << output.trajectory.get_duration()
        << " [s]."
        << std::endl;

    return 0;
}
