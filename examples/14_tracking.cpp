/**
 * <<< Only with Ruckig Pro >>>
 * Description:
 * -------------------------------
 * This example demonstrates how to use Ruckig Pro’s online trajectory generation (OTG)
 * functionality to compute real-time, jerk-limited motion profiles that follow a
 * dynamically changing target state. The core idea is to continuously update the
 * trajectory generator (Trackig) with the desired target position, velocity, and
 * acceleration at each control cycle, while respecting given limits on maximum
 * velocity, acceleration, and jerk. The generator produces smooth trajectories
 * that drive the system from its current state toward the target state without
 * violating any constraints.
 *
 * Use Case:
 * -------------------
 * In industrial robotics, CNC machines, or advanced motion systems (e.g., pick‑and‑place
 * arms, camera gimbals, or 3D printers), the controller must follow a stream of
 * setpoints (e.g., coming from a higher‑level planner or human operator) while
 * ensuring smooth and safe movement. This code can be integrated into a control
 * loop running at a fixed cycle time (here 10 ms) to generate the next position
 * command on the fly, smoothly transitioning between different motion profiles
 * (ramp, constant acceleration, sinusoidal) without stopping or exceeding hardware
 * limits.
 *
 * Features Used:
 * ----------------------------------
 * 1. **Trackig\<DegreesOfFreedom\>**  
 *    - Online Trajectory Generator (OTG) for real‑time updates.  
 *    - Configurable control cycle (`delta_time` = 0.01 s).  
 *    - Adjustable “reactiveness” parameter to blend between strict constraint
 *      enforcement and responsiveness to new targets.
 *
 * 2. **TargetState\<DegreesOfFreedom\>**  
 *    - Encapsulates the desired position, velocity, and acceleration for each axis.
 *    - Three example target models provided:
 *      - `model_ramp`: linear velocity ramp until reaching a set position.
 *      - `model_constant_acceleration`: quadratic position profile with constant acceleration.
 *      - `model_sinus`: smooth sinusoidal motion.
 *
 * 3. **InputParameter\<DegreesOfFreedom\> & OutputParameter\<DegreesOfFreedom\>**  
 *    - Define current state and motion limits (velocity, acceleration, jerk,
 *      and optional position bounds).  
 *    - `pass_to_input()` method carries the OTG output forward as the new input
 *      for the next cycle.
 *
 * 4. **Update Loop**  
 *    - Every cycle, the code:
 *      1. Evaluates the current target state at time t.  
 *      2. Calls `otg.update()` to compute the next feasible state.  
 *      3. Prints target vs. actual position for analysis/logging.  
 *      4. Passes the computed state into the input for the next iteration.
 *
 * Functionality:
 * ----------------------------
 * - `#include <ruckig/trackig.hpp>`: Core header for OTG.  
 * - `Trackig<1> otg(0.01)`: Instantiate a 1‑DoF trajectory generator with 10 ms cycle.  
 * - `input.max_velocity`, `.max_acceleration`, `.max_jerk`: Define system limits.  
 * - `otg.reactiveness`: Adjust how aggressively the generator pursues new targets.  
 * - `otg.update(target_state, input, output)`: Main call to compute motion.  
 * - `output.pass_to_input(input)`: Feed output back into input for continuity.  
 * - Utility function `join(...)` to convert vectors to printable strings.
 *
 * This combination of features makes it ideal for embedding into real‑time
 * control loops where the target trajectory is not known in advance but must
 * be followed smoothly and safely under strict motion constraints.
 */


#include <cmath>
#include <iostream>

#include <ruckig/trackig.hpp>

using namespace ruckig;

// Create the target state signal
TargetState<1> model_ramp(double t, double ramp_vel=0.5, double ramp_pos=1.0) {
    TargetState<1> target;
    const bool on_ramp = t < ramp_pos / std::abs(ramp_vel);
    target.position[0] = on_ramp ? t * ramp_vel : ramp_pos;
    target.velocity[0] = on_ramp ? ramp_vel : 0.0;
    target.acceleration[0] = 0.0;
    return target;
}

TargetState<1> model_constant_acceleration(double t, double ramp_acc=0.05) {
    TargetState<1> target;
    target.position[0] = t * t * ramp_acc;
    target.velocity[0] = t * ramp_acc;
    target.acceleration[0] = ramp_acc;
    return target;
}

TargetState<1> model_sinus(double t, double ramp_vel=0.4) {
    TargetState<1> target;
    target.position[0] = std::sin(ramp_vel * t);
    target.velocity[0] = ramp_vel * std::cos(ramp_vel * t);
    target.acceleration[0] = -ramp_vel * ramp_vel * std::sin(ramp_vel * t);
    return target;
}

int main() {
    // Create instances: the Trackig OTG as well as input and output parameters
    Trackig<1> otg(0.01);  // control cycle
    InputParameter<1> input;
    OutputParameter<1> output;

    // Set input parameters
    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};

    input.max_velocity = {0.8};
    input.max_acceleration = {2.0};
    input.max_jerk = {5.0};

    // Optional minimum and maximum position
    input.min_position = {-2.5};
    input.max_position = {2.5};

    otg.reactiveness = 1.0; // default value, should be in [0, 1]

    // Generate the trajectory following the target state
    std::cout << "target | follow" << std::endl;
    for (size_t t = 0; t < 500; t += 1) {
        const TargetState<1> target_state = model_ramp(otg.delta_time * t);
        const Result res = otg.update(target_state, input, output);
        std::cout << join(target_state.position) << " " << join(output.new_position) << std::endl;

        output.pass_to_input(input);
    }
}
