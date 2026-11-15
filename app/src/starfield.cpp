#include "starfield.hpp"

#include <SFML/Graphics/Color.hpp>

#include <cstdint>
#include <random>

Starfield::Starfield(unsigned int width, unsigned int height, std::size_t count, unsigned int seed) {
    if (count == 0) {
        return;
    }

    positions_.reserve(count);
    shapes_.reserve(count);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist_x(0.0f, static_cast<float>(width));
    std::uniform_real_distribution<float> dist_y(0.0f, static_cast<float>(height));
    std::uniform_real_distribution<float> dist_radius(0.7f, 1.2f);
    std::uniform_int_distribution<int> dist_alpha(100, 255);

    for (std::size_t i = 0; i < count; ++i) {
        float x = dist_x(rng);
        float y = dist_y(rng);
        float radius = dist_radius(rng);
        auto alpha = static_cast<std::uint8_t>(dist_alpha(rng));

        positions_.emplace_back(x, y);

        sf::CircleShape star(radius);
        star.setOrigin({radius, radius});
        star.setPosition({x, y});
        star.setFillColor(sf::Color(255, 255, 255, alpha));
        shapes_.push_back(star);
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
