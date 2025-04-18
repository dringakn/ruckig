/*
    Description:
    This program demonstrates real‑time, jerk‑limited trajectory 
    generation for a 3‑degree‑of‑freedom (3‑DOF) system using the 
    Ruckig library in conjunction with the Eigen linear algebra 
    library.

    At a high level, the code sets up:
      1. A control cycle period (0.01 s) for iterative updates.
      2. Input parameters describing the system’s current and desired (target) state:
         – Position, velocity, and acceleration vectors.
      3. Physical limits for each axis: maximum velocity, acceleration, and jerk.
      4. An online trajectory generator (OTG) instance that, at each control cycle,
         computes the next feasible position, velocity, and acceleration step
         respecting the specified constraints and minimizing travel time.

    Inside a loop, `otg.update(input, output)` advances the trajectory by one time step.
    The newly computed state is printed, and then fed back into the next iteration
    via `output.pass_to_input(input)`. This continues until the trajectory completes,
    at which point the total duration is queried from the generated trajectory.

    Use Case:
    Such code is typically used in high‑performance motion control applications where
    smooth, time‑optimal movements are required under strict dynamic limits. 
    Examples include:
      • Industrial robot arm joint trajectory planning.
      • Collaborative robot (cobot) motion smoothing.
      • CNC machine toolpath execution.
      • Camera‑mounted drone flight path generation.
      • Automated pick‑and‑place gantry systems.

    Features and Functionality:
    ------------------------------------
    • **Template Parameterization (3, EigenVector)**  
      Compile‑time dimension (`3`) and vector type (`Eigen::Vector3d`) abstraction.
    • **Control Cycle Specification**  
      A fixed update period of 0.01 s to drive the real‑time loop.
    • **InputParameter / OutputParameter**  
      Data structures encapsulating current/target states and computed results.
    • **State Definitions**  
      – `current_position`, `current_velocity`, `current_acceleration`  
      – `target_position`, `target_velocity`, `target_acceleration`
    • **Dynamic Constraints**  
      – `max_velocity`, `max_acceleration`, `max_jerk` per axis for jerk‑limited profiling.
    • **Online Trajectory Generation**  
      The `otg.update(...)` method computes the next trajectory segment, returning
      a `Result` status (`Working` until completion).
    • **Feedback Loop**  
      `output.pass_to_input(input)` seamlessly feeds the newly computed state back
      into the next cycle, ensuring continuity.
    • **Trajectory Introspection**  
      `output.trajectory.get_duration()` retrieves the total execution time.

    This structure ensures that at each control interval, the trajectory remains
    within kinematic bounds, producing smooth, safe, and time‑optimal motion.
*/

#include <iostream>

// Include Eigen before Ruckig
#include <Eigen/Core> // Version 3.4 or later

#include <ruckig/ruckig.hpp>

using namespace ruckig;

int main() {
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3, EigenVector> otg(0.01);  // control cycle period in seconds
    InputParameter<3, EigenVector> input;
    OutputParameter<3, EigenVector> output;

    // Define the starting position in 3D space
    Eigen::Vector3d start_position;
    start_position << 0.0, 0.0, 0.5;

    // Define how far to move along each axis
    Eigen::Vector3d position_diff;
    position_diff << 5.0, -2.0, -4.0;

    // Set current state
    input.current_position     = start_position;
    input.current_velocity     = {0.0, -2.2, -0.5};
    input.current_acceleration = {0.0,  2.5, -0.5};

    // Set desired target state
    input.target_position      = start_position + position_diff;
    input.target_velocity      = {0.0, -0.5, -2.0};
    input.target_acceleration  = {0.0,  0.0,  0.5};

    // Define per‑axis kinematic limits
    input.max_velocity     = {3.0, 1.0, 3.0};
    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk         = {4.0, 3.0, 2.0};

    // Run the trajectory generation in a real‑time control loop
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;
        // Feed the new state back for the next control cycle
        output.pass_to_input(input);
    }

    // Report the total trajectory duration
    std::cout << "Trajectory duration: "
              << output.trajectory.get_duration()
              << " [s]." << std::endl;

    return 0;
}
