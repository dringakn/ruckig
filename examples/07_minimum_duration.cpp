/*
 * Description
 *
 * This program demonstrates a real-time, jerk-limited trajectory generation
 * for a 3‑degree‑of‑freedom (3‑DoF) system using the Ruckig Online Trajectory
 * Generation (OTG) library. It computes smooth motion profiles that respect
 * specified limits on velocity, acceleration, and jerk, updating them at each
 * control cycle until the target state is reached.
 *
 * Use Case
 * In robotics and automation (e.g., robotic arms, CNC machines, camera gimbals),
 * motion must be planned online to adapt to sensor feedback or changing targets.
 * This code could be part of a robot controller that, every 10 ms, recalculates
 * the next setpoint to smoothly drive each joint from its current state to the
 * desired one, without exceeding hardware limits or inducing vibrations.
 *
 * Features and Ruckig Functionality
 *  • Ruckig<3> otg(0.01):  
 *      Creates a 3‑DoF OTG instance with a 10 ms control cycle.
 *
 *  • InputParameter<3> / OutputParameter<3>:  
 *      Hold the current and target kinematic states (position, velocity,
 *      acceleration), along with maximum limits (velocity, acceleration, jerk)
 *      and an optional minimum trajectory duration.
 *
 *  • input.minimum_duration = 5.0:  
 *      Enforces a minimum execution time (here 5 s), useful for synchronizing
 *      multi-axis moves or ensuring a minimum sensor integration period.
 *
 *  • otg.update(input, output):  
 *      Advances the trajectory by one control step. Returns Result::Working
 *      until the motion completes, then Result::Finished or an error code.
 *
 *  • output.pass_to_input(input):  
 *      Feeds the newly computed state back as the start for the next cycle,
 *      enabling continuous online replanning.
 *
 *  • output.trajectory.get_duration():  
 *      Retrieves the total time the trajectory will take, accounting for all
 *      constraints and phases.
 *
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3> otg(0.01);  // control cycle of 10 ms
    InputParameter<3> input;
    OutputParameter<3> output;

    // Set current kinematic state (position [m], velocity [m/s], acceleration [m/s²])
    input.current_position     = { 0.0,   0.0,   0.5 };
    input.current_velocity     = { 0.0,  -2.2,  -0.5 };
    input.current_acceleration = { 0.0,   2.5,  -0.5 };

    // Set target kinematic state
    input.target_position      = { -5.0, -2.0,  -3.5 };
    input.target_velocity      = {  0.0, -0.5,  -2.0 };
    input.target_acceleration  = {  0.0,  0.0,   0.5 };

    // Define per‑axis limits (velocity [m/s], acceleration [m/s²], jerk [m/s³])
    input.max_velocity     = { 3.0, 1.0, 3.0 };
    input.max_acceleration = { 3.0, 2.0, 1.0 };
    input.max_jerk         = { 4.0, 3.0, 2.0 };

    // Ensure the trajectory lasts at least this long (synchronization, safety)
    input.minimum_duration = 5.0;

    // Run the control loop: update until the trajectory is complete
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;
        // Prepare for next cycle by making the last output the new input
        output.pass_to_input(input);
    }

    // After completion, report the total planned trajectory duration
    std::cout << "Trajectory duration: "
              << output.trajectory.get_duration() << " [s]." << std::endl;

    return 0;
}
