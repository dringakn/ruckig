/*
 * Description:
 * -----------------------
 * This example demonstrates how to generate a time-optimal trajectory for a multi-degree-of-freedom system
 * using the Ruckig library, incorporating intermediate waypoints. The program:
 *  1. Defines the system's current state (position, velocity, acceleration).
 *  2. Specifies a series of intermediate positions that the trajectory must pass through.
 *  3. Sets target state constraints (position, velocity, acceleration) and motion limits (max velocity, acceleration, jerk).
 *  4. Computes and updates the trajectory in real-time control cycles, printing the position at each time step.
 *
 * Use Case:
 * -------------------
 * Ideal for robotics and automation applications where a manipulator or an actuator must move through
 * multiple precise points in space—such as pick-and-place tasks, tool-path planning in CNC machines,
 * or coordinated multi-axis motion in industrial robots—while ensuring smooth and time-efficient motion.
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    const double control_cycle = 0.01;
    const size_t DOFs = 3;
    const size_t max_number_of_waypoints = 10;  // for memory allocation

    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<DOFs> otg(control_cycle, max_number_of_waypoints);
    InputParameter<DOFs> input;
    OutputParameter<DOFs> output(max_number_of_waypoints);

    // Set input parameters
    input.current_position = {0.2, 0.0, -0.3};
    input.current_velocity = {0.0, 0.2, 0.0};
    input.current_acceleration = {0.0, 0.6, 0.0};

    input.intermediate_positions = {
        {1.4, -1.6, 1.0},
        {-0.6, -0.5, 0.4},
        {-0.4, -0.35, 0.0},
        {0.8, 1.8, -0.1}
    };

    input.target_position = {0.5, 1.0, 0.0};
    input.target_velocity = {0.2, 0.0, 0.3};
    input.target_acceleration = {0.0, 0.1, -0.1};

    input.max_velocity = {1.0, 2.0, 1.0};
    input.max_acceleration = {3.0, 2.0, 2.0};
    input.max_jerk = {6.0, 10.0, 20.0};

    input.interrupt_calculation_duration = 500; // [µs]

    std::cout << "t | position" << std::endl;
    double calculation_duration = 0.0;
    while (otg.update(input, output) == Result::Working) {
        if (output.new_calculation) {
            std::cout << "Updated the trajectory:" << std::endl;
            std::cout << "  Reached target position in " << output.trajectory.get_duration() << " [s]." << std::endl;
            std::cout << "  Calculation in " << output.calculation_duration << " [µs]." << std::endl;
        }

        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        output.pass_to_input(input);
    }
}
