#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/System/Vector2.hpp>

#include <cstddef>
#include <vector>

/**
 * @file starfield.hpp
 * @brief Background starry sky visualization component for the simulator window.
 */

/**
 * @class Starfield
 * @brief Generates and renders a static, deterministic field of stars across the viewport.
 */
class Starfield {
public:
    /**
     * @brief Constructs an empty starfield.
     */
    Starfield() = default;

    /**
     * @brief Generates a pseudo-random starfield bounded by width and height.
     *
     * @param width Viewport width in pixels.
     * @param height Viewport height in pixels.
     * @param count Total number of star points to scatter.
     * @param seed Random generator seed for deterministic generation.
     */
    Starfield(unsigned int width, unsigned int height, std::size_t count = 300, unsigned int seed = 1337);

    /**
     * @brief Renders all stars onto the specified render target.
     * @param target Destination SFML render target (window or render texture).
     */
    void render(sf::RenderTarget& target) const;

    /**
     * @brief Returns the total number of stars in the field.
     * @return std::size_t Star count.
     */
    std::size_t star_count() const;

    /**
     * @brief Returns the collection of 2D star coordinate positions.
     * @return const std::vector<sf::Vector2f>& Vector of positions.
     */
    const std::vector<sf::Vector2f>& positions() const;

private:
    std::vector<sf::Vector2f> positions_;    ///< Star 2D coordinates in screen space.
    std::vector<sf::CircleShape> shapes_;     ///< Pre-configured SFML drawable shapes for rendering.
};
