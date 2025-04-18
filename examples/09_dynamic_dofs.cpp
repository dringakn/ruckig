/*
 * Description:
 * -----------------------
 * This program demonstrates how to generate a smooth, time-optimal trajectory
 * for a multi-dimensional system (e.g., a robotic arm with three joints) using
 * the Ruckig online trajectory generator (OTG). Given the current state
 * (position, velocity, acceleration) and desired target state for each degree
 * of freedom, it computes jerk-limited motion profiles that respect
 * per-axis constraints on maximum velocity, acceleration, and jerk. The
 * trajectory is produced incrementally in a simulated control loop, where at
 * each time step the OTG provides the next position setpoint, which is then
 * fed back as the new "current" state for the following iteration.
 *
 * Use Case:
 * -------------------
 * In practice, this pattern is used in robotic motion control, CNC machines,
 * and any mechatronic system requiring smooth, safe, and efficient movement.
 * For example, controlling a 3‑axis Cartesian robot: you specify the
 * starting pose and velocity, the goal pose and final velocity, and hardware
 * limits; the algorithm continually outputs new setpoints that can be sent
 * to low‑level motor controllers at a fixed control rate (here 100 Hz).
 *
 * Features and Functionality:
 * --------------------------------
 * • DynamicDOFs template parameter: allows specifying degrees of freedom at runtime  
 * • Time step (0.01 s) passed to Ruckig constructor: defines control-loop rate  
 * • InputParameter / OutputParameter: structures holding current/target state and results  
 * • Setting per-axis constraints: max_velocity, max_acceleration, max_jerk  
 * • update() method: computes the next trajectory segment; returns a Result enum  
 * • Result::Working: indicates that the trajectory is still being generated  
 * • output.pass_to_input(input): feeds the last computed state back into input for continuity  
 * • trajectory.get_duration(): retrieves the total duration of the planned motion  
 * • join(): utility to print vector contents as a comma‑separated list  
 */

#include <iostream>
#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Number of independent axes/joints
    size_t degrees_of_freedom = 3;

    // Instantiate the Ruckig online trajectory generator with dynamic DOFs
    // and a control-loop interval of 10 ms.
    Ruckig<DynamicDOFs> otg(degrees_of_freedom, 0.01);
    InputParameter<DynamicDOFs> input(degrees_of_freedom);
    OutputParameter<DynamicDOFs> output(degrees_of_freedom);

    // Initialize the current state (position [m or rad], velocity, acceleration)
    input.current_position     = { 0.0,   0.0,  0.5  };
    input.current_velocity     = { 0.0,  -2.2, -0.5  };
    input.current_acceleration = { 0.0,   2.5, -0.5  };

    // Specify the target state
    input.target_position      = { 5.0,  -2.0, -3.5  };
    input.target_velocity      = { 0.0,  -0.5, -2.0  };
    input.target_acceleration  = { 0.0,   0.0,  0.5  };

    // Apply per-axis motion constraints
    input.max_velocity     = { 3.0, 1.0, 3.0 };
    input.max_acceleration = { 3.0, 2.0, 1.0 };
    input.max_jerk         = { 4.0, 3.0, 2.0 };

    // Run the control loop: repeatedly call update() until the trajectory is complete
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        // Print the current time and interpolated position setpoint
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        // Prepare for next iteration: use the last output as the new input state
        output.pass_to_input(input);
    }

    // Once finished, output the total planned trajectory duration
    std::cout << "Trajectory duration: "
              << output.trajectory.get_duration()
              << " [s]" << std::endl;

    return 0;
}
