#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <string>
#include "../../utilities/utilities.h"

using namespace SimTK;
constexpr double final_time = 5.0;

static void BM_OpenSimPendulum(benchmark::State& st, int numLinks) {

    OpenSim::Model model = createOpenSimPendulum(numLinks);
    SimTK::State state = model.initSystem();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    state.setTime(0);
    timeStepper.initialize(state);
    
    // Run the benchmark.
    for (auto _ : st) {
        timeStepper.stepTo(final_time);
    }
}

static void BM_SimbodyPendulum(benchmark::State& st, int numLinks) {

    PendulumSystem pendulumSystem(numLinks);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    
    for (auto _ : st) {
        ts.stepTo(final_time);
    }
}

int main(int argc, char** argv) {
    
    std::vector<int> numLinksList = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

    // Register the benchmarks.
    for (int numLinks : numLinksList) {
        std::string benchmarkName = "OpenSim_link" + std::to_string(numLinks);
        benchmark::RegisterBenchmark(benchmarkName.c_str(), 
                BM_OpenSimPendulum, numLinks)->Unit(benchmark::kNanosecond);
    }

    for (int numLinks : numLinksList) {
        std::string benchmarkName = "Simbody_link" + std::to_string(numLinks);
        benchmark::RegisterBenchmark(benchmarkName.c_str(), 
                BM_SimbodyPendulum, numLinks)->Unit(benchmark::kNanosecond);
    }

    // Run the benchmarks.
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}