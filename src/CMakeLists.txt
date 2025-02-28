add_executable(realize realize.cpp)
target_link_libraries(realize ${OpenSim_LIBRARIES} benchmark::benchmark)
