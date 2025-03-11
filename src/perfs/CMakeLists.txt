find_package(docopt REQUIRED)

add_executable(perf_realize perf_realize.cpp ../../utilities/utilities.h)
target_link_libraries(perf_realize ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(perf_realize PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(perf_forward perf_forward.cpp ../../utilities/utilities.h)
target_link_libraries(perf_forward ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(perf_forward PRIVATE ${docopt_INCLUDE_DIRS})
