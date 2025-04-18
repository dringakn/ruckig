/*
 * Description:
 *
 * This example demonstrates how to generate a smooth, time-optimal trajectory for a multi‑degree
 *‑of‑freedom (DOF) system while passing through a series of intermediate waypoints. It uses the
 * Ruckig library’s Online Trajectory Generation (OTG) algorithm in its “Pro” or “cloud-enabled”
 * variant, which supports waypoint handling.
 *
 * The program sets up:
 *  1. A control loop timing (control_cycle) at which trajectory updates are computed.
 *  2. The number of DOFs for the system (e.g., a robotic arm with 3 axes).
 *  3. A maximum buffer size for intermediate waypoints, to pre‑allocate memory.
 *
 * In the main routine:
 *  - An OTG object is instantiated with dynamic DOFs, the control cycle, and waypoint capacity.
 *  - InputParameter and OutputParameter structures are created for feeding state and retrieving
 *    trajectory data.
 *  - The current state (position, velocity, acceleration) is defined.
 *  - A list of intermediate_positions (waypoints) is provided, through which the trajectory must pass.
 *  - The final target state (position, velocity, acceleration) is defined.
 *  - Motion limits are set for each axis: maximum velocity, acceleration, and jerk.
 *
 * The control loop repeatedly:
 *  - Calls otg.update(input, output) to compute the next segment of the trajectory.
 *  - Prints the current time and positions.
 *  - Feeds the newly computed state back into input for the next iteration.
 *  - Tracks how long each calculation took when a new trajectory segment is computed.
 *
 * Once the trajectory reaches the final target, the total trajectory duration and the last
 * calculation time are reported.
 *
 * Use Case:
 *  This pattern is typical in real‑time control of robotic manipulators, CNC machines, or
 *  autonomous vehicles, where you need to plan a time‑optimal path through predefined
 *  waypoints (for obstacle avoidance, precision motion, or complex path following) under
 *  strict kinematic limits.
 *
 * Features:
 *  • Intermediate waypoints support (requires Ruckig Pro or cloud API)  
 *  • Dynamic multi‑DOF handling via template parameter DynamicDOFs  
 *  • Real‑time trajectory update loop with fixed control cycle  
 *  • Per‑axis motion constraints: max velocity, acceleration, and jerk  
 *  • Passing output back to input to maintain continuity  
 *  • Measurement of computation duration for performance monitoring  
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    double control_cycle = 0.01;
    size_t DOFs = 3;
    size_t max_number_of_waypoints = 10;  // for memory allocation

    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<DynamicDOFs> otg(DOFs, control_cycle, max_number_of_waypoints);
    InputParameter<DynamicDOFs> input(DOFs);
    OutputParameter<DynamicDOFs> output(DOFs, max_number_of_waypoints);

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

    std::cout << "t | position" << std::endl;
    double calculation_duration = 0.0;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        output.pass_to_input(input);

        if (output.new_calculation) {
            calculation_duration = output.calculation_duration;
        }
    }

    std::cout << "Reached target position in " << output.trajectory.get_duration() << " [s]." << std::endl;
    std::cout << "Calculation in " << calculation_duration << " [µs]." << std::endl;
}
