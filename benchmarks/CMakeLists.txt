find_package(docopt REQUIRED)

add_executable(realize realize.cpp)
target_link_libraries(realize ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(realize PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(forward forward.cpp utilities.h)
target_link_libraries(forward ${OpenSim_LIBRARIES} benchmark::benchmark docopt_s)
target_include_directories(forward PRIVATE ${docopt_INCLUDE_DIRS})