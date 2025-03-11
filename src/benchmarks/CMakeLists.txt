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