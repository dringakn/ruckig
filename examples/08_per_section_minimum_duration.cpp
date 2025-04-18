/*
  Description
  This example demonstrates how to perform real-time, jerk-limited trajectory
  planning with multiple intermediate waypoints for a 3‑degree‑of‑freedom (DOF)
  system using the Ruckig library (Pro or cloud API required for intermediate
  waypoint support). It configures the current state, a sequence of intermediate
  target positions, and final target conditions, then iteratively computes
  motion commands at a fixed control cycle until the final target is reached.

  Use Case
  - Industrial robotics: guiding a robotic arm through several key poses (e.g.,
    pick‑up, inspection, placement) while respecting velocity, acceleration,
    and jerk limits.
  - CNC machines or camera cranes: smoothly transitioning through user‑defined
    waypoints to avoid abrupt motions and ensure precision.
  - Automated guided vehicles (AGVs): planning a path through checkpoints while
    guaranteeing comfort and mechanical safety.

  Features
  1. **Online Trajectory Generation (OTG)**  
     Generates setpoints in real time at each control cycle (10 ms here).
  2. **Intermediate Waypoints**  
     Allows specifying multiple intermediate positions that the trajectory must
     pass through before reaching the final target.
  3. **Per‑Section Minimum Duration**  
     Enforces a minimum time for each segment (between waypoints) to control
     timing and synchronization.
  4. **Multi‑DOF Support**  
     Handles coordinated planning for three independent axes (or joints).
  5. **Kinematic Constraints**  
     Applies individual limits on maximum velocity, acceleration, and jerk for
     each DOF to ensure smoothness and safety.
  6. **Pre‑Allocation of Memory**  
     Reserves space for up to 10 waypoints to avoid dynamic allocations during
     real‑time execution.

*/

#include <iostream>
#include <vector>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    const double control_cycle = 0.01;           // 10 ms loop time
    const size_t DOFs = 3;                       // Three axes/joints
    const size_t max_number_of_waypoints = 10;   // Memory reserved for waypoints

    // Instantiate the Ruckig OTG engine and parameter structures
    Ruckig<DOFs> otg(control_cycle, max_number_of_waypoints);
    InputParameter<DOFs> input;
    OutputParameter<DOFs> output(max_number_of_waypoints);

    // Current state of the system
    input.current_position     = {0.8,  0.0,  0.5};
    input.current_velocity     = {0.0,  0.0,  0.0};
    input.current_acceleration = {0.0,  0.0,  0.0};

    // A sequence of waypoints the trajectory must pass through
    input.intermediate_positions = {
        { 1.4, -1.6,  1.0},
        {-0.6, -0.5,  0.4},
        {-0.4, -0.35, 0.0},
        {-0.2,  0.35,-0.1},
        { 0.2,  0.5, -0.1},
        { 0.8,  1.8, -0.1}
    };

    // Final target state
    input.target_position     = {0.5,  1.2, 0.0};
    input.target_velocity     = {0.0,  0.0, 0.0};
    input.target_acceleration = {0.0,  0.0, 0.0};

    // Kinematic limits for each DOF
    input.max_velocity     = {3.0, 2.0, 2.0};
    input.max_acceleration = {6.0, 4.0, 4.0};
    input.max_jerk         = {16.0,10.0,20.0};

    // Minimum duration for each trajectory segment (0 = unconstrained)
    // Size = number_of_waypoints + 1 = 6 waypoints + final segment = 7 entries
    input.per_section_minimum_duration = {0.0, 2.0, 0.0, 1.0, 0.0, 2.0, 0.0};

    std::cout << "t | position" << std::endl;
    double last_calculation_duration = 0.0;

    // Execute the trajectory online until completion
    while (otg.update(input, output) == Result::Working) {
        std::cout
            << output.time << " | "
            << join(output.new_position) << std::endl;

        // Feed the newly computed state back into the input for the next cycle
        output.pass_to_input(input);

        // Capture computation time when a new segment starts
        if (output.new_calculation) {
            last_calculation_duration = output.calculation_duration;
        }
    }

    // Summary of the executed trajectory
    std::cout << "Reached target position in "
              << output.trajectory.get_duration()
              << " [s]." << std::endl;
    std::cout << "Last calculation took "
              << last_calculation_duration
              << " [µs]." << std::endl;

    return 0;
}
