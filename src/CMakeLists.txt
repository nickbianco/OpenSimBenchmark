add_executable(realize realize.cpp)
target_link_libraries(realize ${OpenSim_LIBRARIES} benchmark::benchmark)

file(GLOB MODEL_FILES ${CMAKE_SOURCE_DIR}/models/**/RajagopalLaiUhlrich2023*.osim)
file(COPY ${MODEL_FILES} DESTINATION ${CMAKE_BINARY_DIR}/src)
