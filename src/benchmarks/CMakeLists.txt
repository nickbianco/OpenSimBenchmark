find_package(docopt REQUIRED)

add_executable(benchmark_realize benchmark_realize.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_realize ${OpenSim_LIBRARIES} docopt_s)
target_include_directories(benchmark_realize PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_forward_rk4 benchmark_forward_rk4.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_forward_rk4 ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_forward_rk4 PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_forward_euler benchmark_forward_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_forward_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_forward_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_manager_rk4 benchmark_manager_rk4.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_manager_rk4 ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_manager_rk4 PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_manager_euler benchmark_manager_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_manager_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_manager_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(opensim_vs_simbody_pendulum opensim_vs_simbody_pendulum.cpp ../../utilities/utilities.h)
target_link_libraries(opensim_vs_simbody_pendulum ${OpenSim_LIBRARIES} docopt_s)
target_include_directories(opensim_vs_simbody_pendulum PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(integrator_comparison integrator_comparison.cpp ../../utilities/utilities.h)
target_link_libraries(integrator_comparison ${OpenSim_LIBRARIES} docopt_s)
target_include_directories(integrator_comparison PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_simbody_pendulum_euler benchmark_simbody_pendulum_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_simbody_pendulum_rk4 benchmark_simbody_pendulum_rk4.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum_rk4 ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum_rk4 PRIVATE ${docopt_INCLUDE_DIRS})