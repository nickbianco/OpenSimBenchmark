#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a simulation of an N-link pendulum in Simbody.

Usage:
  perf_forward <nlinks> <output> [--time <time>] [--step <step>]
  perf_forward -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
  -s <step>, --step <step>        Set the step size.
)";

void resetState(SimTK::State& state) {
    // Reset the state to the initial state.
    state.setTime(0);
    state.updQ() = SimTK::Vector(state.getNQ(), 0.0);
    state.updQ()[0] = 1.0;
}

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

    // Pendulum system.
    int numLinks = std::stoi(args["<nlinks>"].asString());
    OPENSIM_THROW_IF(numLinks <= 0, OpenSim::Exception, 
            "Number of links must be positive.");
    PendulumSystem pendulum(numLinks);

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
        OPENSIM_THROW_IF(step <= 0, OpenSim::Exception, 
                "Step size must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j;

    // Create the initial state.
    const SimTK::MultibodySystem& system = pendulum.m_system;
    SimTK::State state = system.realizeTopology();

    // Realize acceleration.
    // ---------------------
    resetState(state);
    auto start = high_resolution_clock::now();
    system.realize(state, SimTK::Stage::Acceleration);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;
    j["acceleration_compute_time"] = time_elapsed;

    // One simulation step.
    // --------------------
    resetState(state);
    SimTK::SemiExplicitEulerIntegrator integrator(system, step);
    SimTK::TimeStepper timeStepper(system, integrator);
    timeStepper.initialize(state);
    start = high_resolution_clock::now();
    timeStepper.stepTo(step);
    end = high_resolution_clock::now();
    time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;
    j["single_step_time"] = time_elapsed;

    // Forward integration.
    // --------------------
    resetState(state);
    integrator.resetAllStatistics();
    timeStepper.initialize(state);
    start = high_resolution_clock::now();
    timeStepper.stepTo(time);
    end = high_resolution_clock::now();
    time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;
    j["forward_integration_time"] = time_elapsed;
    j["real_time_factor"] = time / time_elapsed;

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}