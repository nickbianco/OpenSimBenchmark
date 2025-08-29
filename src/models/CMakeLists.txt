add_executable(gait3d gait3d.cpp ../../utilities/utilities.h)
target_link_libraries(gait3d ${OpenSim_LIBRARIES} docopt_s nlohmann_json::nlohmann_json)
target_include_directories(gait3d PRIVATE ${docopt_INCLUDE_DIRS})
