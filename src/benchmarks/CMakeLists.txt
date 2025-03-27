find_package(docopt REQUIRED)

add_executable(benchmark_realize benchmark_realize.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_realize ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(benchmark_realize PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_forward benchmark_forward.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_forward ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(benchmark_forward PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_manager benchmark_manager.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_manager ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(benchmark_manager PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(opensim_vs_simbody_pendulum opensim_vs_simbody_pendulum.cpp ../../utilities/utilities.h)
target_link_libraries(opensim_vs_simbody_pendulum ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(opensim_vs_simbody_pendulum PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(integrator_comparison integrator_comparison.cpp ../../utilities/utilities.h)
target_link_libraries(integrator_comparison ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(integrator_comparison PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_simbody_pendulum benchmark_simbody_pendulum.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum PRIVATE ${docopt_INCLUDE_DIRS})