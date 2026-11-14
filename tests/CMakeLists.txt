add_executable(test_body test_body.cpp)
target_link_libraries(test_body PRIVATE nbody-physics)

add_executable(test_integrators test_integrators.cpp)
target_link_libraries(test_integrators PRIVATE nbody-physics)

add_executable(test_physics test_physics.cpp)
target_link_libraries(test_physics PRIVATE nbody-physics)

add_executable(test_presets test_presets.cpp ../app/src/sim_presets.cpp)
target_include_directories(test_presets PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_presets PRIVATE nbody-physics)
