#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a simulation of a 3D gait model in Simbody.

Usage:
  benchmark_gait3d_hunt_crossley_euler <output> --time=<time> --step=<step> --accuracy=<accuracy>
  benchmark_gait3d_hunt_crossley_euler -h | --help

Options:
  -t <time>,      --time <time>            Set the final time.
  -s <step>,      --step <step>            Set the step size.
  -a <accuracy>,  --accuracy <accuracy>    Set the integrator accuracy.
)";

int main(int argc, char** argv) {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;
    
    // Disable logging.
    OpenSim::Logger::setLevelString("error");

    // Parse the command line arguments.
    std::map<std::string, docopt::value> args = parse_arguments(
        HELP, { argv + 1, argv + argc },
        true); // show help if requested

    // 3D gait system.
    Gait3D model(Gait3D::ContactType::CompliantContactSubsystem);

    // Final time.
    double time = 5.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception, 
                "Final time must be positive.");
    }

    // Step size.
    double step = -1; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
    }

    // Integrator accuracy.
    double accuracy = 0.001;
    if (args["--accuracy"]) {
        accuracy = std::stod(args["--accuracy"].asString());
        OPENSIM_THROW_IF(accuracy <= 0, OpenSim::Exception, 
                "Integrator accuracy must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j;

    // Create the initial state.
    const SimTK::MultibodySystem& system = model.m_system;
    SimTK::State state = system.realizeTopology();

        // Realize acceleration.
    // ---------------------
    SimTK::Vector acceleration_times(100, 0.0);
    for (int i = 0; i < 100; ++i) {
        model.loadDefaultState(state);
        auto start = high_resolution_clock::now();
        system.realize(state, SimTK::Stage::Acceleration);
        auto end = high_resolution_clock::now();
        double time_elapsed = duration_cast<microseconds>(end - start).count();
        time_elapsed /= 1.0e6;
        acceleration_times[i] = time_elapsed;
    }
    j["acceleration_compute_time"] = SimTK::mean(acceleration_times);

    // One simulation step.
    // --------------------
    if (step > 0) {
        SimTK::SemiExplicitEuler2Integrator integrator(system);
        integrator.setAccuracy(accuracy);
        integrator.setFixedStepSize(step);
        SimTK::TimeStepper timeStepper(system, integrator);
        SimTK::Vector step_times(100, 0.0);
        for (int i = 0; i < 100; ++i) {
            model.loadDefaultState(state);
            timeStepper.initialize(state);
            auto start = high_resolution_clock::now();
            timeStepper.stepTo(step);
            auto end = high_resolution_clock::now();
            double time_elapsed = duration_cast<microseconds>(end - start).count();
            time_elapsed /= 1.0e6;
            step_times[i] = time_elapsed;
        }
        j["single_step_time"] = SimTK::mean(step_times);
    }

    // Initial energy
    // --------------
    model.loadDefaultState(state);
    system.realize(state, SimTK::Stage::Dynamics);
    double initial_energy = system.calcEnergy(state);

    // Forward integration.
    // --------------------
    SimTK::Vector integration_times(10, 0.0);
    SimTK::Vector real_time_factors(10, 0.0);
    SimTK::Vector final_energies(10, 0.0);
    for (int i = 0; i < 10; ++i) {
        SimTK::SemiExplicitEuler2Integrator integrator(system);
        integrator.setAccuracy(accuracy);
        if (step > 0) {
            integrator.setFixedStepSize(step);
        }
        SimTK::TimeStepper timeStepper(system, integrator);
        model.loadDefaultState(state);
        timeStepper.initialize(state);
        auto start = high_resolution_clock::now();
        timeStepper.stepTo(time);
        auto end = high_resolution_clock::now();
        double time_elapsed = duration_cast<microseconds>(end - start).count();
        time_elapsed /= 1.0e6;
        integration_times[i] = time_elapsed;
        real_time_factors[i] = time / time_elapsed;

        // Final energy
        // ------------
        const auto& finalState = timeStepper.getState();
        system.realize(finalState, SimTK::Stage::Dynamics);
        final_energies[i] = system.calcEnergy(finalState);
    }
    j["forward_integration_time"] = SimTK::mean(integration_times);
    j["real_time_factor"] = SimTK::mean(real_time_factors);
    double final_energy = SimTK::mean(final_energies);
    j["energy_conservation"] = final_energy - initial_energy;

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}