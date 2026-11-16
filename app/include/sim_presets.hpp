#ifndef SIM_PRESETS
#define SIM_PRESETS

#include <vector>
#include <random>
#include "body.hpp"
#include "sim_config.hpp"


/**
 * @file sim_presets.hpp
 * @brief Factory functions for initializing standard gravitational simulation scenarios.
 */

namespace presets {

/**
 * @brief High-contrast neon color palette used for random cloud particle visualization.
 */
extern const std::vector<ColorRGB> kNeonPalette;

/**
 * @brief Creates a deterministic two-body stable circular orbit scenario.
 *
 * Places a heavy central mass (M = 10000) at the coordinate center and an orbiting mass
 * (m = 1) at distance d = 200 with perpendicular circular orbital velocity:
 * \f$ v = \sqrt{\frac{G \cdot M}{d}} \f$.
 *
 * @param cfg Simulation configuration providing G and window dimensions for centering.
 * @return std::vector<Body> Vector containing the central body and orbiting body.
 */
std::vector<Body> create_stable_orbit(const SimConfig& cfg);

/**
 * @brief Creates a multi-body solar system scenario with exponential orbital spacing.
 *
 * Initializes a central sun and 5 planetary bodies (Mercury, Venus, Earth, Mars, Jupiter)
 * with distances scaled according to astronomical unit (AU) ratios, realistic mass/radius
 * hierarchies, and circular orbital velocities.
 *
 * @param cfg Simulation configuration providing G and window dimensions.
 * @return std::vector<Body> Vector containing Sun and 5 planets.
 */
std::vector<Body> create_solar_system(const SimConfig& cfg);

/**
 * @brief Generates a pseudo-random particle cloud scenario.
 *
 * Scatters 80 particles uniformly within 90% of the window bounds with randomized
 * masses, radii proportional to \f$ \sqrt[3]{m} \f$, and bounded velocities.
 *
 * @param cfg Simulation configuration providing window dimensions.
 * @param seed Pseudo-random generator seed for deterministic generation.
 * @return std::vector<Body> Vector of 80 randomly distributed bodies.
 */


std::vector<Body> create_random_cloud(const SimConfig& cfg, unsigned int seed = -1);

} // namespace presets


#endif