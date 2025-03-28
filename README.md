# OpenSimBenchmark
Performance benchmarking for OpenSim models and simulations.


### Updating `perf_event_paranoid` and `kptr_restrict`

echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid

echo 0 | sudo tee /proc/sys/kernel/kptr_restrict


### VSCode workspace settings

```
{
    "cmake.configureSettings": {
        "OpenSim_DIR": "<workspace>/OpenSimBenchmark/dependencies/opensim/opensim_core_install/cmake",
        "Simbody_DIR": "<workspace>/OpenSimBenchmark/dependencies/opensim/opensim_dependencies_install/simbody/lib/cmake/simbody",
        "benchmark_DIR": "<workspace>/OpenSimBenchmark/dependencies/benchmark/benchmark_install/lib/cmake/benchmark",
        "docopt_DIR": "<workspace>/OpenSimBenchmark/dependencies/opensim/opensim_dependencies_install/docopt/lib/cmake/docopt",
        "docopt_INCLUDE_DIRS": "<workspace>/OpenSimBenchmark/dependencies/opensim/opensim_dependencies_install/docopt/include",
    },
}
```
