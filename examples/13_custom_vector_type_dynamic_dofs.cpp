/*
  Description
  This program demonstrates how to perform real-time, jerk-limited trajectory
  generation for a system with multiple degrees of freedom (DOFs) using the
  Ruckig online trajectory generator (OTG) library. The core idea is to
  compute smooth position, velocity, and acceleration profiles that respect
  specified maximum velocity, acceleration, and jerk limits, updating these
  profiles at each control cycle.

  The flow is as follows:
    1. Define a minimal container (`MinimalDynamicDofsVector`) to hold state
       and limit values for a dynamic number of DOFs, using a `std::deque`
       internally for flexible resizing.
    2. Instantiate the Ruckig OTG for a given number of DOFs (here: 3) and a
       control cycle time (here: 10 ms).
    3. Populate the input parameters:
         • Current position, velocity, acceleration
         • Target position, velocity, acceleration
         • Maximum allowed velocity, acceleration, jerk for each axis/DOF
    4. Enter a control loop:
         • Call `otg.update(input, output)` to compute the next step of the
           trajectory. While the trajectory is still in progress (`Working`),
           print the current time and computed position.
         • Feed the newly computed state back into the input for the next
           cycle, enabling continuous real-time updates.
    5. When the trajectory is complete, output the total trajectory duration.

  Use Case
  -------------------
  • Industrial Robotics: Planning smooth arm or end‑effector motions that
    avoid abrupt changes in acceleration (jerk), protecting both the payload
    and mechanical components from excessive stress.
  • CNC Machining & 3D Printing: Generating toolpaths that adhere to speed,
    acceleration, and jerk constraints for precision and surface finish.
  • Autonomous Vehicles & Drones: Computing safe, comfortable trajectories
    for multi‑axis motion (e.g., pan‑tilt cameras, gimbals) in real time.
  • Any embedded or control application requiring low‑latency trajectory
    updates within a fixed control loop.

  Features
  -----------------
  1. **Template Metaprogramming**  
     - `MinimalDynamicDofsVector<T, DOFs>`: Generic container supporting
       an arbitrary number of DOFs at runtime, while keeping compile‑time
       template parameters for library compatibility.
  2. **Standard Library Components**  
     - `std::deque<T>`: Dynamically resizable buffer for storing per‑DOF data.
     - `std::initializer_list<T>`: Convenient initialization of vectors
       (e.g., `{0.0, -2.2, -0.5}`).
     - Operator overloading (`operator[]`, `operator==`) for intuitive usage.
  3. **Ruckig Library**  
     - `Ruckig<DynamicDOFs, MinimalDynamicDofsVector>`: The OTG engine,
       parameterized for dynamic DOFs and custom vector type.
     - `InputParameter` & `OutputParameter`: Typed structs to specify
       current/target states and retrieve computed next‑step states.
     - `update()`: Computes the trajectory incrementally, returning status
       flags (`Working`, `Finished`, `Error`).
     - `pass_to_input()`: Utility to feed the previous output directly as the
       next input, simplifying control‑loop logic.
  4. **Real‑Time Control Loop**  
     - Fixed control cycle (`0.01` s) to meet deterministic update rates.
     - Streaming output to `std::cout` for logging or visualization.

*/

#include <deque>
#include <iostream>

#include <ruckig/ruckig.hpp>


template<class T, size_t DOFs>
class MinimalDynamicDofsVector {
    std::deque<T> data;

public:
    MinimalDynamicDofsVector() { }
    MinimalDynamicDofsVector(std::initializer_list<T> a) {
        data.resize(a.size());
        std::copy_n(a.begin(), a.size(), std::begin(data));
    }

    T operator[](size_t i) const {
        return data[i];
    }

    T& operator[](size_t i) {
        return data[i];
    }

    size_t size() const {
        return data.size();
    }

    void resize(size_t size) {
        data.resize(size);
    }

    bool operator==(const MinimalDynamicDofsVector<T, DOFs>& rhs) const {
        for (size_t dof = 0; dof < data.size(); ++dof) {
            if (data[dof] != rhs[dof]) {
                return false;
            }
        }
        return true;
    }
};


using namespace ruckig;

int main() {
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<DynamicDOFs, MinimalDynamicDofsVector> otg(3, 0.01);  // control cycle
    InputParameter<DynamicDOFs, MinimalDynamicDofsVector> input(3);
    OutputParameter<DynamicDOFs, MinimalDynamicDofsVector> output(3);

    // Set input parameters
    input.current_position = {0.0, 0.0, 0.5};
    input.current_velocity = {0.0, -2.2, -0.5};
    input.current_acceleration = {0.0, 2.5, -0.5};

    input.target_position = {5.0, -2.0, -3.5};
    input.target_velocity = {0.0, -0.5, -2.0};
    input.target_acceleration = {0.0, 0.0, 0.5};

    input.max_velocity = {3.0, 1.0, 3.0};
    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk = {4.0, 3.0, 2.0};

    // Generate the trajectory within the control loop
    std::cout << "t | position" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        std::cout << output.time << " | " << join(output.new_position) << std::endl;

        output.pass_to_input(input);
    }

    std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;
}
