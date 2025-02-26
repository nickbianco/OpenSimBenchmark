add_executable(realize realize.cpp)
target_link_libraries(realize ${OpenSim_LIBRARIES} benchmark::benchmark)

file(COPY ${CMAKE_SOURCE_DIR}/models/RajagopalLaiUhlrich2023.osim
        DESTINATION ${CMAKE_BINARY_DIR}/src)