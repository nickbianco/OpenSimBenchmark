# OpenSimBenchmark
Performance benchmarking for OpenSim models and simulations.


### Updating `perf_event_paranoid` and `kptr_restrict`

echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid

echo 0 | sudo tee /proc/sys/kernel/kptr_restrict


### VSCode workspace settings example

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

### `config.yaml` example

```
results_path: <workspace>/OpenSimBenchmark/results
figures_path: <workspace>/OpenSimBenchmark/figures
data_path: <workspace>/OpenSimBenchmark/data
mujoco_path: <workspace>/OpenSimBenchmark/mujoco
dependencies_path: <workspace>/OpenSimBenchmark/dependencies
benchmarks_path: <workspace>/OpenSimBenchmark/build/src/benchmarks
models_path: <workspace>/OpenSimBenchmark/models
```