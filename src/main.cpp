#include <benchmark/benchmark.h>
#include <OpenSim/OpenSim.h>
#include <Simbody.h>

static void BM_LoadModel(benchmark::State& state) {
    for (auto _ : state) {
        try {
            OpenSim::Model model("RajagopalLaiUhlrich2023.osim"); // Replace with an actual model
        } catch (const std::exception& e) {
            state.SkipWithError(e.what());
        }
    }
}

BENCHMARK(BM_LoadModel)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
