find_package(docopt REQUIRED)

add_executable(sandbox sandbox.cpp ../../utilities/utilities.h)
target_link_libraries(sandbox ${OpenSim_LIBRARIES} osimMoco docopt_s)
target_include_directories(sandbox PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(sandbox_pendulum sandbox_pendulum.cpp ../../utilities/utilities.h)
target_link_libraries(sandbox_pendulum ${OpenSim_LIBRARIES} osimMoco docopt_s)
target_include_directories(sandbox_pendulum PRIVATE ${docopt_INCLUDE_DIRS})

add_executable(sandbox_contact sandbox_contact.cpp ../../utilities/utilities.h)
target_link_libraries(sandbox_contact ${OpenSim_LIBRARIES})

add_executable(sandbox_jaredsdude sandbox_jaredsdude.cpp ../../utilities/utilities.h)
target_link_libraries(sandbox_jaredsdude ${OpenSim_LIBRARIES})

