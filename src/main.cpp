#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>

class LoadModel : public benchmark::Fixture {
public:
    OpenSim::Model model;
    SimTK::State state;

    void SetUp(const ::benchmark::State& st) override {
        OpenSim::Logger::setLevelString("off");
        model = OpenSim::Model("RajagopalLaiUhlrich2023.osim");
        state = model.initSystem();
    }
};

BENCHMARK_DEFINE_F(LoadModel, BM_RealizePosition)(benchmark::State& st) {
    for (auto _ : st) {
        state.updY() = SimTK::Test::randVector(state.getY().size());
        model.realizePosition(state);
    }
}

BENCHMARK_DEFINE_F(LoadModel, BM_RealizeVelocity)(benchmark::State& st) {
    for (auto _ : st) {
        state.updY() = SimTK::Test::randVector(state.getY().size());
        model.realizeVelocity(state);
    }
}

BENCHMARK_DEFINE_F(LoadModel, BM_RealizeDynamics)(benchmark::State& st) {
    for (auto _ : st) {
        state.updY() = SimTK::Test::randVector(state.getY().size());
        model.realizeDynamics(state);
    }
}

BENCHMARK_DEFINE_F(LoadModel, BM_RealizeAcceleration)(benchmark::State& st) {
    for (auto _ : st) {
        state.updY() = SimTK::Test::randVector(state.getY().size());
        model.realizeAcceleration(state);
    }
}

BENCHMARK_REGISTER_F(LoadModel, BM_RealizePosition)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(LoadModel, BM_RealizeVelocity)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(LoadModel, BM_RealizeDynamics)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(LoadModel, BM_RealizeAcceleration)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();