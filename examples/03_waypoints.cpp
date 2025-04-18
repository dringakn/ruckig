/**
 * Description:
 * -----------------------
 * In real‑world scenarios—like a 3‑axis robotic arm performing a pick‑and‑place with
 * obstacle‑avoidance waypoints, or a small UAV navigating through inspection points—
 * you often need smooth, jerk‑limited motion that passes through several intermediate
 * positions before reaching the final target. This example shows how to configure
 * Ruckig to:
 *
 * 1. Define your robot’s current state (position, velocity, acceleration).
 * 2. Specify a sequence of intermediate waypoints to visit in order.
 * 3. Set the final target state (position, velocity, acceleration).
 * 4. Enforce per‑axis limits on velocity, acceleration and jerk.
 * 5. Run the online trajectory generator at 100 Hz, automatically re‑computing
 *    when a new segment is needed and logging the computation time.
 *
 * The result: a continuous S‑curve trajectory through all waypoints, safe for hardware,
 * with known execution and calculation timing—ideal for real‑time control loops.
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Control parameters
    const double control_cycle = 0.01;      // 10 ms update interval
    const size_t DOFs = 3;                  // three independent axes
    const size_t max_waypoints = 10;        // reserve space for up to 10 waypoints

    // Instantiate Ruckig with intermediate‑waypoint support
    Ruckig<DOFs> otg(control_cycle, max_waypoints);
    InputParameter<DOFs>  input;
    OutputParameter<DOFs> output(max_waypoints);

    // --- 1) Define current state of the system ---
    input.current_position     = { 0.2,  0.0, -0.3 };
    input.current_velocity     = { 0.0,  0.2,  0.0 };
    input.current_acceleration = { 0.0,  0.6,  0.0 };

    // --- 2) List intermediate waypoints (e.g. inspection or approach poses) ---
    input.intermediate_positions = {
        { 1.4, -1.6,  1.0 },
        { -0.6, -0.5, 0.4 },
        { -0.4, -0.35, 0.0 },
        { 0.8,  1.8, -0.1 }
    };

    // --- 3) Set final target state ---
    input.target_position     = { 0.5,  1.0,  0.0 };
    input.target_velocity     = { 0.2,  0.0,  0.3 };
    input.target_acceleration = { 0.0,  0.1, -0.1 };

    // --- 4) Configure per‑axis limits ---
    input.max_velocity     = { 1.0,  2.0,  1.0 };
    input.max_acceleration = { 3.0,  2.0,  2.0 };
    input.max_jerk         = { 6.0, 10.0, 20.0 };

    std::cout << "t [s] | position [x, y, z]\n";
    double calc_time_us = 0.0;

    // --- 5) Control loop: generate next segment until done ---
    while (otg.update(input, output) == Result::Working) {
        // Print time and new position
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        // Feed the new state back for the next cycle
        output.pass_to_input(input);

        // When Ruckig recalculates (e.g. at a waypoint), record the compute time
        if (output.new_calculation) {
            calc_time_us = output.calculation_duration;
        }
    }

    // Final reporting
    std::cout << "Total trajectory duration: "
              << output.trajectory.get_duration() << " s\n";
    std::cout << "Last recalculation took: "
              << calc_time_us << " µs\n";

    return 0;
}
