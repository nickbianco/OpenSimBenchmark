cmake_minimum_required(VERSION 3.10)
project(OpenSimBenchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the path to the locally built OpenSim
set(OpenSim_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/opensim/opensim_core_install/cmake"
        CACHE PATH "Path to OpenSim installation" FORCE)
set(spdlog_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/opensim/opensim_dependencies_install/spdlog/lib/cmake/spdlog"
        CACHE PATH "Path to spdlog installation" FORCE)

# Find OpenSim, Simbody, and Google Benchmark
find_package(OpenSim REQUIRED CONFIG)

# Fetch nlohmann::json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

FetchContent_Declare(docopt URL https://github.com/docopt/docopt.cpp/archive/refs/tags/v0.6.3.tar.gz)
FetchContent_MakeAvailable(docopt)

# Add source directory
add_subdirectory(src)
