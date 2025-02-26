#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <string>
#include "utilities.h"

static void realize(const SimTK::Stage& stage, benchmark::State& st, 
        OpenSim::Model& model, SimTK::State& state, 
        StateGenerator& generator) {
    // Get a random state vector.
    SimTK::Vector y = generator.getRandomY();
    // Get a random control vector.
    SimTK::Vector controls = generator.getRandomControls();
    // Set the controls in the model.
    model.realizeVelocity(state);
    model.setControls(state, controls);
    // Run the benchmark.
    for (auto _ : st) {
        state.updY() = y;
        model.getSystem().realize(state, stage);
    }
}

static void BM_RealizePosition(benchmark::State& st, OpenSim::Model& model, 
        SimTK::State& state, StateGenerator& generator) {
    realize(SimTK::Stage::Position, st, model, state, generator);
}

static void BM_RealizeVelocity(benchmark::State& st, OpenSim::Model& model, 
        SimTK::State& state, StateGenerator& generator) {
    realize(SimTK::Stage::Velocity, st, model, state, generator);
}

static void BM_RealizeDynamics(benchmark::State& st, OpenSim::Model& model, 
        SimTK::State& state, StateGenerator& generator) {
    realize(SimTK::Stage::Dynamics, st, model, state, generator);
}

static void BM_RealizeAcceleration(benchmark::State& st, OpenSim::Model& model, 
        SimTK::State& state, StateGenerator& generator) {
    realize(SimTK::Stage::Acceleration, st, model, state, generator);
}

int main(int argc, char** argv) {
    
    // Disable logging.
    OpenSim::Logger::setLevelString("error");

    // Load the model and initialize the system.
    OpenSim::Model model(argv[1]);
    SimTK::State state = model.initSystem();

    // Create a state generator.
    StateGenerator generator(model);
    generator.printBounds();

    // Register the benchmarks.
    benchmark::RegisterBenchmark("realizePosition", BM_RealizePosition, 
            std::ref(model), 
            std::ref(state), 
            std::ref(generator))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeVelocity", BM_RealizeVelocity,
            std::ref(model), 
            std::ref(state), 
            std::ref(generator))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeDynamics", BM_RealizeDynamics,
            std::ref(model), 
            std::ref(state), 
            std::ref(generator))->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("realizeAcceleration", BM_RealizeAcceleration,
            std::ref(model), 
            std::ref(state), 
            std::ref(generator))->Unit(benchmark::kMillisecond);

    // Run the benchmarks.
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
}