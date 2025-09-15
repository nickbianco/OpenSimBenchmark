#include <OpenSim/OpenSim.h>
#include "../../utilities/utilities.h"

int main(int argc, char** argv) {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;

    // 3D gait system.
    Gait3D model(Gait3D::ContactType::ExponentialSpringForce);


    // Create the initial state.
    SimTK::MultibodySystem& system = model.m_system;
    // Set up visualization.
    // system.setUseUniformBackground(false);
    // model.m_matter.setShowDefaultGeometry(false);
    // Visualizer viz(system);
    // system.addEventReporter(new Visualizer::Reporter(viz, 0.01));

    SimTK::State state = system.realizeTopology();

    // Forward integration.
    // --------------------
    SimTK::CPodesIntegrator integrator(system);
    integrator.setAccuracy(0.01);

    double time = 5.0; // seconds
    SimTK::TimeStepper timeStepper(system, integrator);
    model.loadDefaultState(state);
    timeStepper.initialize(state);
    auto start = high_resolution_clock::now();
    timeStepper.stepTo(time);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;


    std::cout << "Time elapsed: " << time_elapsed << " seconds\n";
    std::cout << "Real-time factor: " << time / time_elapsed << "\n";
}