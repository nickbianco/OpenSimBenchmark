find_package(docopt REQUIRED)

add_executable(sandbox sandbox.cpp ../../utilities/utilities.h)
target_link_libraries(sandbox ${OpenSim_LIBRARIES} osimMoco benchmark::benchmark docopt_s)
target_include_directories(sandbox PRIVATE ${docopt_INCLUDE_DIRS})


