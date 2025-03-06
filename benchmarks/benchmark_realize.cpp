#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <string>
#include "../utilities/utilities.h"

static const char HELP[] =
R"(Benchmark realizing an OpenSim::Model to a SimTK::Stage.

Usage:
  forward <model> [--randomize <bool>]
  forward -h | --help

Options:
  -r <bool>, --randomize <bool>   Randomize the state.
)";

static void realize(const SimTK::Stage& stage, const StateGenerator& generator, 
        benchmark::State& st, OpenSim::Model& model, SimTK::State& state,
        bool randomize) {

    state = model.initSystem();
    SimTK::Vector y = state.getY();
    if (randomize) {
        // Get a random state vector.
        y = generator.getRandomY();

        // Get a random control vector.
        SimTK::Vector controls = generator.getRandomControls();
        model.realizeVelocity(state);
        model.setControls(state, controls);  
    }

    // Run the benchmark.
    for (auto _ : st) {
        state.updY() = y;
        model.getSystem().realize(state, stage);
    }
}

static void BM_RealizePosition(benchmark::State& st, 
        const StateGenerator& generator, OpenSim::Model& model, 
        SimTK::State& state, bool randomize) {
    realize(SimTK::Stage::Position, generator, st, model, state, randomize);
}

static void BM_RealizeVelocity(benchmark::State& st, 
        const StateGenerator& generator, OpenSim::Model& model, 
        SimTK::State& state, bool randomize) {
    realize(SimTK::Stage::Velocity, generator, st, model, state, randomize);
}

static void BM_RealizeDynamics(benchmark::State& st, 
        const StateGenerator& generator, OpenSim::Model& model, 
        SimTK::State& state, bool randomize) {
    realize(SimTK::Stage::Dynamics, generator, st, model, state, randomize);
}

static void BM_RealizeAcceleration(benchmark::State& st, 
        const StateGenerator& generator, OpenSim::Model& model, 
        SimTK::State& state, bool randomize) {
    realize(SimTK::Stage::Acceleration, generator, st, model, state, randomize);
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

    bool randomize = false;
    if (args["--randomize"]) {
        randomize = (bool)std::stod(args["--randomize"].asString());
        std::cout << "Randomize initial state: " << randomize << std::endl;
    }


    // Create a state generator.
    StateGenerator generator(model);
    // generator.printBounds();

    // Register the benchmarks.
    benchmark::RegisterBenchmark("realizePosition", BM_RealizePosition, 
            std::ref(generator),
            std::ref(model), 
            std::ref(state), randomize)->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeVelocity", BM_RealizeVelocity,
            std::ref(generator),
            std::ref(model), 
            std::ref(state), randomize)->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeDynamics", BM_RealizeDynamics,
            std::ref(generator),
            std::ref(model), 
            std::ref(state), randomize)->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeAcceleration", BM_RealizeAcceleration,
            std::ref(generator),
            std::ref(model), 
            std::ref(state), randomize)->Unit(benchmark::kMillisecond);

    // Run the benchmarks.
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
}