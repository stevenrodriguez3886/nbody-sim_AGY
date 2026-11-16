#pragma once

#include <cstdint>
#include <string>

/**
 * @file sim_config.hpp
 * @brief Configuration parameters for physics simulation and window initialization.
 */

/**
 * @struct SimConfig
 * @brief Holds tunable constants, algorithmic selections, and window geometry.
 */
struct SimConfig {
    float G = 1000.0f;                     ///< Gravitational constant scaling pairwise force magnitude.
    float epsilon = 5.0f;               ///< Softening parameter (pixels) avoiding singularities at r -> 0.
    float dt = 0.01f;                   ///< Fixed integration timestep (seconds) per physics step.
    unsigned int window_width = 1280;   ///< Initial window width in screen pixels.
    unsigned int window_height = 720;   ///< Initial window height in screen pixels.
    std::string integrator = "rk4";   ///< Chosen numerical integrator ("euler" or "rk4").
    std::string preset = "orbit";       ///< Initial simulation preset ("orbit", "solar", or "random").
    uint64_t next_id = 1;               ///< Seed ID counter for generating unique particle identifiers.
};
