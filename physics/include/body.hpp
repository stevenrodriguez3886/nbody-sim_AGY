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
 * @brief Merges two bodies upon collision into a single composite body.
 *
 * Implements a completely inelastic collision conserving linear momentum and mass:
 * - Total mass: \f$ m_{new} = m_a + m_b \f$
 * - Center of mass position: \f$ \mathbf{r}_{new} = \frac{m_a \mathbf{r}_a + m_b \mathbf{r}_b}{m_{new}} \f$
 * - Momentum-conserved velocity: \f$ \mathbf{v}_{new} = \frac{m_a \mathbf{v}_a + m_b \mathbf{v}_b}{m_{new}} \f$
 * - Combined radius conserving 2D surface area: \f$ R_{new} = \sqrt{R_a^2 + R_b^2} \f$
 * - ID inheritance: Inherits the ID of the more massive progenitor (or smaller ID if tied).
 *
 * @param a First colliding body.
 * @param b Second colliding body.
 * @return Body The resulting coalesced body.
 */
Body merge(const Body& a, const Body& b);

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
