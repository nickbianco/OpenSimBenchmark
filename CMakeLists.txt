cmake_minimum_required(VERSION 3.10)
project(OpenSimBenchmark)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenSim, Simbody, and Google Benchmark
find_package(OpenSim REQUIRED CONFIG)
find_package(benchmark REQUIRED)

# Fetch nlohmann::json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# Add source directory
add_subdirectory(src)
