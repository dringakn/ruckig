/*
 * Description:
 * -----------------------
 * This program demonstrates how to generate a time‐optimal, jerk‐limited trajectory
 * for a system with three degrees of freedom (e.g., a 3‐axis robot arm or a 3‐joint
 * mechanism) using the Ruckig online trajectory generator library. It repeatedly
 * computes the next state (position, velocity, acceleration) at each control cycle
 * until the motion constraint (target velocity and acceleration) is reached.
 *
 * In each cycle:
 *   1. The current state and target constraints are provided to the OTG.
 *   2. The OTG computes the new state that respects maximum acceleration and jerk limits.
 *   3. The new state is output for feed‐forwarding to the motion controller.
 *   4. The new state becomes the "current" state for the next cycle.
 *
 * Use Case:
 * -------------------
 * In industrial robotics or CNC machines, smooth and precise motion is critical.
 * This code can be used in a real‐time control loop to:
 *   • Decelerate or accelerate robot joints safely without exceeding mechanical limits.
 *   • Plan multi‐axis coordinated moves under velocity, acceleration, and jerk constraints.
 *   • Ensure smooth transitions to avoid vibrations or mechanical stress.
 *
 * Key Features:
 * ------------------
 * • Ruckig<3>: Instantiates a 3‐DOF online trajectory generator with a 10 ms control cycle.
 * • InputParameter<3> / OutputParameter<3>: Data structures to hold current and output states.
 * • ControlInterface::Velocity: Specifies that motion planning is based on achieving target velocities.
 * • input.current_* & input.target_*: Setting initial and goal states for position, velocity, acceleration.
 * • input.max_acceleration / input.max_jerk: Defining actuator limits for safe motion.
 * • otg.update(...): Computes the next trajectory point; returns Result::Working while motion is ongoing.
 * • output.pass_to_input(input): Transfers newly computed state to become the next cycle’s input.
 * • output.trajectory.get_duration(): Queries the total duration of the planned trajectory.
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3> otg(0.01);  // control cycle of 10 ms
    InputParameter<3> input;
    OutputParameter<3> output;

    // Set input parameters and velocity control interface
    input.control_interface = ControlInterface::Velocity;

    input.current_position = {0.0, 0.0, 0.5};
    input.current_velocity = {3.0, -2.2, -0.5};
    input.current_acceleration = {0.0, 2.5, -0.5};

    input.target_velocity = {0.0, -0.5, -1.5};
    input.target_acceleration = {0.0, 0.0, 0.5};

    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk = {6.0, 6.0, 4.0};

    // Generate the trajectory within the control loop
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        // Feed the output back as the next input state
        output.pass_to_input(input);
    }

    std::cout << "Trajectory duration: "
              << output.trajectory.get_duration()
              << " [s]." << std::endl;
}
