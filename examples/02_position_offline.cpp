/**
 * Description:
 * --------------------------------
 * This program demonstrates **offline** trajectory planning for a 3‑DoF system  
 * (e.g., a small robotic arm, CNC gantry, or drone gimbal) using Ruckig’s jerk‑limited  
 * S‑curve profiles. Instead of stepping through a control loop, we compute the entire  
 * motion in one go, then query any timepoint for position, velocity, and acceleration.  
 * 
 * Use Case:
 *   • A pick‑and‑place robot needs to move its end‑effector from one pose to another  
 *     with strict smoothness constraints to protect delicate payloads.  
 *   • You precompute the motion offline (during setup or when targets update)  
 *     and then stream the resulting trajectory to your controller or simulator.  
 */

#include <iostream>
#include <array>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // 1) Fill input state: current (pos, vel, accel) and target (pos, vel, accel)
    InputParameter<3> input;
    input.current_position     = {0.0,  0.0,  0.5};
    input.current_velocity     = {0.0, -2.2, -0.5};
    input.current_acceleration = {0.0,  2.5, -0.5};

    input.target_position      = {5.0, -2.0, -3.5};
    input.target_velocity      = {0.0, -0.5, -2.0};
    input.target_acceleration  = {0.0,  0.0,  0.5};

    // 2) Define symmetric limits (max) and asymmetric limits (min) per axis
    input.max_velocity     = { 3.0,  1.0,  3.0};
    input.max_acceleration = { 3.0,  2.0,  1.0};
    input.max_jerk         = { 4.0,  3.0,  2.0};

    input.min_velocity     = {-2.0, -0.5, -3.0};
    input.min_acceleration = {-2.0, -2.0, -2.0};

    // 3) Offline trajectory generation (no real‑time loop)
    Ruckig<3> otg;                // default constructor for offline use
    Trajectory<3> trajectory;     // holds the full time‑parameterized profile

    // 4) Compute entire trajectory
    Result result = otg.calculate(input, trajectory);
    if (result == Result::ErrorInvalidInput) {
        std::cerr << "Invalid input parameters!" << std::endl;
        return -1;
    }

    // 5) Inspect trajectory duration
    double duration = trajectory.get_duration();
    std::cout << "Trajectory duration: " << duration << " s\n";

    // 6) Query kinematic state at an arbitrary time
    double query_time = 1.0;  // seconds into the motion
    std::array<double,3> pos, vel, acc;
    trajectory.at_time(query_time, pos, vel, acc);
    std::cout << "At t=" << query_time << " s → pos: " << join(pos)
              << ", vel: " << join(vel)
              << ", acc: " << join(acc) << "\n";

    // 7) Get position extrema over the entire trajectory for each axis
    auto extrema = trajectory.get_position_extrema();
    std::cout << "Axis 3 position range: ["
              << extrema[2].min << ", " << extrema[2].max << "]\n";

    return 0;
}
