# OpenSimBenchmark
Performance benchmarking for OpenSim models and simulations.


### Updating `perf_event_paranoid` and `kptr_restrict`

echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid

echo 0 | sudo tee /proc/sys/kernel/kptr_restrict

