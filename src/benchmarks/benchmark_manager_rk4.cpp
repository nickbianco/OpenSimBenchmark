#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a forward simulation.

Usage:
  benchmark_manager_rk4 <model> <output> --time=<time> --step=<step>
  benchmark_manager_rk4 -h | --help

Options:
  -t <time>, --time <time>  Set the final time.
  -s <step>, --step <step>  Set the step size.
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

    // Model.
    OPENSIM_THROW_IF(!args["<model>"], OpenSim::Exception, 
        "Model file must be provided.");
    OpenSim::Model model(args["<model>"].asString());
    SimTK::State state = model.initSystem();
    const SimTK::MultibodySystem& system = model.getMultibodySystem();

    // Helper function to reset the state.
    Vector defaultY = state.getY();
    auto resetState = [&](SimTK::State& state) {
        state.setTime(0);
        state.updY() = defaultY;
    };

    // Final time.
    double time = -1.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception, 
                "Final time must be positive.");
    }

    // Step size.
    double step = -1.0; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
        OPENSIM_THROW_IF(step <= 0, OpenSim::Exception, 
                "Step size must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j; 

    // Realize acceleration.
    // ---------------------
    SimTK::Vector acceleration_times(100, 0.0);
    for (int i = 0; i < 100; ++i) {
        resetState(state);
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
        SimTK::Vector step_times(100, 0.0);
        for (int i = 0; i < 100; ++i) {
            resetState(state);
            Manager manager(model);
            manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaMerson);
            manager.setIntegratorMinimumStepSize(step);
            manager.setIntegratorMaximumStepSize(step);
            manager.initialize(state);
            auto start = high_resolution_clock::now();
            manager.integrate(step);
            auto end = high_resolution_clock::now();
            double time_elapsed = duration_cast<microseconds>(end - start).count();
            time_elapsed /= 1.0e6;
            step_times[i] = time_elapsed;
        }
        j["single_step_time"] = SimTK::mean(step_times);
    }

    // Initial energy
    // --------------
    resetState(state);
    system.realize(state, SimTK::Stage::Dynamics);
    double initial_energy = system.calcEnergy(state);

    // Forward integration.
    // --------------------
    SimTK::Vector integration_times(10, 0.0);
    SimTK::Vector real_time_factors(10, 0.0);
    SimTK::Vector final_energies(10, 0.0);
    for (int i = 0; i < 10; ++i) {
        resetState(state);
        Manager manager(model);
        manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaMerson);
        if (step > 0) {
            manager.setIntegratorMinimumStepSize(step);
            manager.setIntegratorMaximumStepSize(step);
        }
        manager.initialize(state);
        auto start = high_resolution_clock::now();
        manager.integrate(time);
        auto end = high_resolution_clock::now();
        double time_elapsed = duration_cast<microseconds>(end - start).count();
        time_elapsed /= 1.0e6;
        integration_times[i] = time_elapsed;
        real_time_factors[i] = time / time_elapsed;
        
        // Final energy
        // ------------
        const auto& finalState = manager.getState();
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