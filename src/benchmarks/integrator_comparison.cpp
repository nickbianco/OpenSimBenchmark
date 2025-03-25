#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <string>
#include "../../utilities/utilities.h"

using namespace SimTK;
constexpr double final_time = 2.0;
constexpr int num_links = 5;
constexpr double step_size = 1e-3;

static void BM_RungeKuttaMersonIntegrator(benchmark::State& st) {

    PendulumSystem pendulumSystem(num_links);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    RungeKuttaMersonIntegrator integ(system);
    integ.setFixedStepSize(step_size);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    
    for (auto _ : st) {
        ts.stepTo(final_time);
    }
}

static void BM_SemiExplicitEulerIntegrator(benchmark::State& st) {

    PendulumSystem pendulumSystem(num_links);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    SemiExplicitEulerIntegrator integ(system, step_size);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    
    for (auto _ : st) {
        ts.stepTo(final_time);
    }
}

static void BM_ExplicitEulerIntegrator(benchmark::State& st) {

    PendulumSystem pendulumSystem(num_links);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    ExplicitEulerIntegrator integ(system);
    integ.setFixedStepSize(step_size);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    
    for (auto _ : st) {
        ts.stepTo(final_time);
    }
}

static void BM_VerletIntegrator(benchmark::State& st) {

    PendulumSystem pendulumSystem(num_links);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    VerletIntegrator integ(system);
    integ.setFixedStepSize(step_size);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    
    for (auto _ : st) {
        ts.stepTo(final_time);
    }
}

int main(int argc, char** argv) {
    
    // Register the benchmarks
    benchmark::RegisterBenchmark("RungeKuttaMersonIntegrator", 
        BM_RungeKuttaMersonIntegrator)->Unit(benchmark::kNanosecond);

    benchmark::RegisterBenchmark("SemiExplicitEulerIntegrator",
        BM_SemiExplicitEulerIntegrator)->Unit(benchmark::kNanosecond);

    benchmark::RegisterBenchmark("ExplicitEulerIntegrator",
        BM_ExplicitEulerIntegrator)->Unit(benchmark::kNanosecond);

    benchmark::RegisterBenchmark("VerletIntegrator",
        BM_VerletIntegrator)->Unit(benchmark::kNanosecond);

    // Run the benchmarks.
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}