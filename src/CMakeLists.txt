add_executable(main main.cpp)
target_link_libraries(main ${OpenSim_LIBRARIES} benchmark::benchmark)

# Copy the model file models/RajagopalLaiUhlrich2023.osim to the build directory
file(COPY ${CMAKE_SOURCE_DIR}/models/RajagopalLaiUhlrich2023.osim
        DESTINATION ${CMAKE_BINARY_DIR}/src)