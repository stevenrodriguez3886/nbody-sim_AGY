#pragma once

#include <cstdint>
#include <SFML/System/Vector2.hpp>

/**
 * @file body.hpp
 * @brief Definition of celestial body representation and collision/merge primitives.
 */

/**
 * @struct ColorRGB
 * @brief Simple 24-bit RGB color representation independent of graphics libraries.
 */
struct ColorRGB {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;
};

/**
 * @struct Body
 * @brief Represents a single point-mass celestial body in 2D space.
 */
struct Body {
    uint64_t id = 0;            ///< Unique identifier for the body.
    float mass = 0.0f;          ///< Inertial and gravitational mass (arbitrary units).
    sf::Vector2f position;      ///< Current position vector in physics coordinates (pixels).
    sf::Vector2f velocity;      ///< Current velocity vector in physics coordinates (pixels/second).
    float radius = 0.0f;        ///< Physical and visual collision radius (pixels).
    ColorRGB color = {255, 255, 255}; ///< Base RGB color (defaults to white).

    Body() = default;
    Body(uint64_t id_, float mass_, sf::Vector2f pos_, sf::Vector2f vel_, float rad_, ColorRGB col_ = {255, 255, 255})
        : id(id_), mass(mass_), position(pos_), velocity(vel_), radius(rad_), color(col_) {}
};

/**
 * @brief Resolves a fully elastic 2D collision between two circular bodies.
 *
 * Implements hard-sphere (billiard ball) collision dynamics:
 * - Momentum and kinetic energy conservation: Calculates 2D elastic impulse along
 *   the collision normal to update particle velocities.
 * - Tangential velocity components remain unaffected.
 * - Particle positions are not artificially translated, avoiding non-physical potential energy injection.
 * - Preserves body identities, masses, colors, and radii without merging.
 *
 * @param a First colliding body (modified in-place: velocity).
 * @param b Second colliding body (modified in-place: velocity).
 */
void resolve_elastic_collision(Body& a, Body& b);

/**
 * @brief Checks whether two bodies overlap geometrically in 2D space.
 *
 * Evaluates the Euclidean distance against the sum of the radii:
 * \f$ \|\mathbf{r}_b - \mathbf{r}_a\| < (R_a + R_b) \f$.
 *
 * @param a First body.
 * @param b Second body.
 * @return true if the bodies overlap (collision detected), false otherwise.
 */
bool check_collision(const Body& a, const Body& b);
