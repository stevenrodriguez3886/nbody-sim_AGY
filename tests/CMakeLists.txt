add_executable(test_body test_body.cpp)
target_link_libraries(test_body PRIVATE nbody-physics)
add_test(NAME test_body COMMAND test_body)

add_executable(test_integrators test_integrators.cpp)
target_link_libraries(test_integrators PRIVATE nbody-physics)
add_test(NAME test_integrators COMMAND test_integrators)

add_executable(test_physics test_physics.cpp)
target_link_libraries(test_physics PRIVATE nbody-physics)
add_test(NAME test_physics COMMAND test_physics)

add_executable(test_presets test_presets.cpp ../app/src/sim_presets.cpp)
target_include_directories(test_presets PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_presets PRIVATE nbody-physics)
add_test(NAME test_presets COMMAND test_presets)

add_executable(test_starfield test_starfield.cpp ../app/src/starfield.cpp)
target_include_directories(test_starfield PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_starfield PRIVATE sfml-graphics sfml-window sfml-system)
add_test(NAME test_starfield COMMAND test_starfield)

add_executable(test_menu_bar test_menu_bar.cpp ../app/src/menu_bar.cpp)
target_include_directories(test_menu_bar PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_menu_bar PRIVATE sfml-graphics sfml-window sfml-system)
add_test(NAME test_menu_bar COMMAND test_menu_bar)
