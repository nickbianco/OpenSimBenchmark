cmake_minimum_required(VERSION 3.10)
project(OpenSimBenchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenSim, Simbody, and Google Benchmark
find_package(OpenSim REQUIRED CONFIG)
find_package(benchmark REQUIRED)

# Add source directory
add_subdirectory(benchmarks)
