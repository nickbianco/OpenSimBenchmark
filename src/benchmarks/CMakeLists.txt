find_package(docopt REQUIRED)

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

add_executable(benchmark_simbody_pendulum_euler benchmark_simbody_pendulum_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_simbody_pendulum_rk4 benchmark_simbody_pendulum_rk4.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum_rk4 ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum_rk4 PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_simbody_pendulum_rk4_custom benchmark_simbody_pendulum_rk4_custom.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_simbody_pendulum_rk4_custom ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_simbody_pendulum_rk4_custom PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_forward_cpodes benchmark_forward_cpodes.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_forward_cpodes ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_forward_cpodes PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_gait3d_exponential_spring_euler benchmark_gait3d_exponential_spring_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_gait3d_exponential_spring_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_gait3d_exponential_spring_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_gait3d_hunt_crossley_euler benchmark_gait3d_hunt_crossley_euler.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_gait3d_hunt_crossley_euler ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_gait3d_hunt_crossley_euler PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_gait3d_exponential_spring_cpodes benchmark_gait3d_exponential_spring_cpodes.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_gait3d_exponential_spring_cpodes ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_gait3d_exponential_spring_cpodes PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(benchmark_gait3d_hunt_crossley_cpodes benchmark_gait3d_hunt_crossley_cpodes.cpp ../../utilities/utilities.h)
target_link_libraries(benchmark_gait3d_hunt_crossley_cpodes ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(benchmark_gait3d_hunt_crossley_cpodes PRIVATE ${docopt_INCLUDE_DIRS})