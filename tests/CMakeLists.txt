add_executable(test_body test_body.cpp)
target_link_libraries(test_body PRIVATE nbody-physics)

add_executable(test_integrators test_integrators.cpp)
target_link_libraries(test_integrators PRIVATE nbody-physics)

add_executable(test_physics test_physics.cpp)
target_link_libraries(test_physics PRIVATE nbody-physics)
