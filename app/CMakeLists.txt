add_executable(nbody-sim src/main.cpp)

target_include_directories(nbody-sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(nbody-sim PRIVATE
    nbody-physics
    sfml-graphics
    sfml-window
    sfml-system
)
