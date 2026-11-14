# Static library target for physics engine
# Sources will be added by later tasks; physics_placeholder.cpp is used for the skeleton
add_library(nbody-physics STATIC
    src/body.cpp
    src/euler_integrator.cpp
    src/rk4_integrator.cpp
)

set_target_properties(nbody-physics PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(nbody-physics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Physics depends only on sfml-system for sf::Vector2f
target_link_libraries(nbody-physics PUBLIC
    SFML::System
)
