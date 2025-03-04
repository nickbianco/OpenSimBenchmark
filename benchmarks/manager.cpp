#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <string>
#include "utilities.h"

static const char HELP[] =
R"(Benchmark a forward simulation using OpenSim's Manager.

Usage:
  manager <model> [--time <time>] [--step <step>] [--randomize <bool>]
  manager -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
  -s <step>, --step <step>        Set the step size.
  -r <bool>, --randomize <bool>   Randomize the initial state.
)";

static void BM_Manager(benchmark::State& st, const StateGenerator& generator, 
        OpenSim::Model& model, SimTK::State& state,
        double time, double step, bool randomize) {

    if (randomize) {
        // Get a random state vector.
        state.updY() = generator.getRandomY();

        // Get a random control vector.
        SimTK::Vector controls = generator.getRandomControls();
        model.realizeVelocity(state);
        model.setControls(state, controls);  
    }

    state.setTime(0);

    Manager manager(model);
    if (step > 0) {
        manager.setIntegratorMinimumStepSize(step);
        manager.setIntegratorMaximumStepSize(step);
    }
    manager.initialize(state);
    
    // Run the benchmark.
    for (auto _ : st) {
        manager.integrate(time);
    }
}

int main(int argc, char** argv) {
    
    // Disable logging.
    OpenSim::Logger::setLevelString("error");

    // Parse the command line arguments.
    std::map<std::string, docopt::value> args = parse_arguments(
        HELP, { argv + 1, argv + argc - 1 },
        true); // show help if requested

    // Model.
    OPENSIM_THROW_IF(!args["<model>"], OpenSim::Exception, 
            "Model file must be provided.");
    OpenSim::Model model(args["<model>"].asString());
    SimTK::State state = model.initSystem();

    // Final time and step size.
    double time = 1.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception, 
                "Final time must be positive.");
    }

    double step = -1; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
        OPENSIM_THROW_IF(step <= 0, OpenSim::Exception, 
                "Step size must be positive.");
    }

    bool randomize = false;
    if (args["--randomize"]) {
        randomize = (bool)std::stod(args["--randomize"].asString());
        std::cout << "Randomize initial state: " << randomize << std::endl;
    }

    // Create a state generator.
    StateGenerator generator(model);
    // generator.printBounds();

    // Register the benchmarks.
    benchmark::RegisterBenchmark("manager", BM_Manager, 
            std::ref(generator),
            std::ref(model), 
            std::ref(state),
            time, step, randomize)->Unit(benchmark::kMillisecond);

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
}