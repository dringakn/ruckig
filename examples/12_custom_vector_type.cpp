/*
 * Description:
 * -------------------------------
 * This program demonstrates online trajectory generation (OTG) for a 3-degree-of-freedom (DOF)
 * system using the Ruckig library. Its purpose is to compute a time‑parameterized, jerk‑limited
 * (S‑curve) motion profile that moves from a current state (position, velocity, acceleration)
 * to a target state while enforcing user‑defined maximum velocity, acceleration, and jerk limits.
 *
 * Use Case:
 * -------------------
 * Such trajectory generation is critical in robotics, CNC machining, pick‑and‑place systems,
 * and autonomous vehicles—any application requiring smooth, precise motion between waypoints
 * without exceeding mechanical or safety constraints. For example, moving a robotic arm between
 * assembly points while minimizing vibration and stress on the joints.
 *
 * Features:
 * --------------------------
 * 1. MinimalVector<T, DOFs>:
 *    - A lightweight, fixed‑size container template to hold per‑DOF state data.
 *    - Supports initialization via std::initializer_list, element access (operator[]),
 *      size(), and comparison (operator==).
 *
 * 2. Ruckig<DOFs, Container>:
 *    - Instantiates the Ruckig OTG algorithm for 3 DOFs, using MinimalVector as the
 *      underlying data structure.
 *    - The constructor argument (0.01) sets the control‑loop time step (10 ms update rate).
 *
 * 3. InputParameter & OutputParameter:
 *    - InputParameter packs the current and target states along with kinematic limits:
 *      • current_position, current_velocity, current_acceleration
 *      • target_position,  target_velocity,  target_acceleration
 *      • max_velocity,    max_acceleration,    max_jerk
 *    - OutputParameter receives the computed next state (new_position, new_velocity,
 *      new_acceleration) and the associated trajectory time.
 *
 * 4. Online Control Loop:
 *    - The call otg.update(input, output) computes the next trajectory segment.
 *    - While it returns Result::Working, the trajectory is still being generated:
 *        • Print output.time and output.new_position each cycle.
 *        • Invoke output.pass_to_input(input) to feed back the last state as the new
 *          “current” state, ensuring a continuous, streaming trajectory.
 *
 * 5. Trajectory Completion:
 *    - Once otg.update no longer returns Working, the full trajectory is complete.
 *    - The total duration is retrieved via output.trajectory.get_duration().
 *
 * 6. Utility:
 *    - join(output.new_position) is a helper to format the MinimalVector contents for printing.
 *
 * Overall, this example sets up, executes, and monitors a smooth, constraint‑respecting motion
 * profile in a real‑time loop—typical for embedded robotic controllers or motion‑control software.
 */

// Standard I/O
#include <iostream>

// Ruckig online trajectory generation library
#include <ruckig/ruckig.hpp>


// A minimal fixed‑size vector container to satisfy Ruckig’s container interface
template<class T, size_t DOFs>
class MinimalVector {
    T data[DOFs];

public:
    MinimalVector() { }
    MinimalVector(std::initializer_list<T> a) {
        std::copy_n(a.begin(), DOFs, std::begin(data));
    }

    T operator[](size_t i) const {
        return data[i];
    }

    T& operator[](size_t i) {
        return data[i];
    }

    size_t size() const {
        return DOFs;
    }

    bool operator==(const MinimalVector<T, DOFs>& rhs) const {
        for (size_t dof = 0; dof < DOFs; ++dof) {
            if (data[dof] != rhs[dof]) {
                return false;
            }
        }
        return true;
    }
};


using namespace ruckig;

int main() {
    // Instantiate the OTG for 3 DOFs with a 10 ms control cycle
    Ruckig<3, MinimalVector> otg(0.01);
    InputParameter<3, MinimalVector> input;
    OutputParameter<3, MinimalVector> output;

    // Define the current state
    input.current_position     = { 0.0,  0.0,  0.5 };
    input.current_velocity     = { 0.0, -2.2, -0.5 };
    input.current_acceleration = { 0.0,  2.5, -0.5 };

    // Define the target state
    input.target_position      = { 5.0, -2.0, -3.5 };
    input.target_velocity      = { 0.0, -0.5, -2.0 };
    input.target_acceleration  = { 0.0,  0.0,  0.5 };

    // Set kinematic limits
    input.max_velocity     = { 3.0, 1.0, 3.0 };
    input.max_acceleration = { 3.0, 2.0, 1.0 };
    input.max_jerk         = { 4.0, 3.0, 2.0 };

    // Online trajectory generation loop
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;
        // Feed the last computed state back as the new "current" state
        output.pass_to_input(input);
    }

    // Report total trajectory duration
    std::cout << "Trajectory duration: " 
              << output.trajectory.get_duration() 
              << " [s]." << std::endl;
}
