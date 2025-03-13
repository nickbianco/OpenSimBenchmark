#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"

static const char HELP[] =
R"(Benchmark realizing an OpenSim::Model to a SimTK::Stage.

Usage:
  forward <model>
  forward -h | --help

Options:
)";

static void realize(const SimTK::Stage& stage,
        benchmark::State& st, OpenSim::Model& model, 
        StateGenerator& stateGenerator,
        SimTK::State& state) {

    SimTK::Vector controls = stateGenerator.getRandomControls();
    model.realizeVelocity(state);
    model.setControls(state, controls);
    SimTK::Vector y = stateGenerator.getRandomY();

    // Run the benchmark.
    for (auto _ : st) {
        state.updY() = y;
        model.getSystem().realize(state, stage);
    }
}

static void BM_RealizePosition(benchmark::State& st, OpenSim::Model& model, 
        StateGenerator& stateGenerator, SimTK::State& state) {
    realize(SimTK::Stage::Position, st, model, stateGenerator, state);
}

static void BM_RealizeVelocity(benchmark::State& st, OpenSim::Model& model, 
        StateGenerator& stateGenerator, SimTK::State& state) {
    realize(SimTK::Stage::Velocity, st, model, stateGenerator, state);
}

static void BM_RealizeDynamics(benchmark::State& st, OpenSim::Model& model, 
        StateGenerator& stateGenerator, SimTK::State& state) {
    realize(SimTK::Stage::Dynamics, st, model, stateGenerator, state);
}

static void BM_RealizeAcceleration(benchmark::State& st, OpenSim::Model& model, 
        StateGenerator& stateGenerator, SimTK::State& state) {
    realize(SimTK::Stage::Acceleration, st, model, stateGenerator, state);
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

    // State generator.
    StateGenerator stateGenerator(model);

    // Register the benchmarks.
    benchmark::RegisterBenchmark("realizePosition", BM_RealizePosition, 
            std::ref(model), 
            std::ref(stateGenerator),   
            std::ref(state))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeVelocity", BM_RealizeVelocity,
            std::ref(model),
            std::ref(stateGenerator), 
            std::ref(state))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeDynamics", BM_RealizeDynamics,
            std::ref(model), 
            std::ref(stateGenerator), 
            std::ref(state))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeAcceleration", BM_RealizeAcceleration,
            std::ref(model), 
            std::ref(stateGenerator), 
            std::ref(state))->Unit(benchmark::kMillisecond);

    // Run the benchmarks.
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
}