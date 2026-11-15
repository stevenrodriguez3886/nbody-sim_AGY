#include "starfield.hpp"

#include <SFML/Graphics/Color.hpp>

#include <cstdint>
#include <random>
#include <utility>

/**
 * @file starfield.cpp
 * @brief Implementation of the background starfield generator and renderer.
 */

Starfield::Starfield(unsigned int width, unsigned int height, std::size_t count, unsigned int seed) {
    if (count == 0) {
        return;
    }

    positions_.reserve(count);
    shapes_.reserve(count);

    // Deterministic Mersenne Twister engine
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist_x(0.0f, static_cast<float>(width));
    std::uniform_real_distribution<float> dist_y(0.0f, static_cast<float>(height));
    std::uniform_real_distribution<float> dist_radius(0.7f, 1.2f);
    std::uniform_int_distribution<int> dist_alpha(100, 255); // Alpha range for subtle visual depth

    for (std::size_t i = 0; i < count; ++i) {
        const float x = dist_x(rng);
        const float y = dist_y(rng);
        const float radius = dist_radius(rng);
        const auto alpha = static_cast<std::uint8_t>(dist_alpha(rng));

        positions_.emplace_back(x, y);

        // Configure pre-baked circular dot primitive
        sf::CircleShape star(radius);
        star.setOrigin({radius, radius});
        star.setPosition({x, y});
        star.setFillColor(sf::Color(255, 255, 255, alpha));
        shapes_.push_back(std::move(star));
    }
}

void Starfield::render(sf::RenderTarget& target) const {
    for (const auto& shape : shapes_) {
        target.draw(shape);
    }
}

std::size_t Starfield::star_count() const {
    return positions_.size();
}

const std::vector<sf::Vector2f>& Starfield::positions() const {
    return positions_;
}
