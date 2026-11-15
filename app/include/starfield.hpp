#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/System/Vector2.hpp>

#include <cstddef>
#include <vector>

class Starfield {
public:
    Starfield() = default;
    Starfield(unsigned int width, unsigned int height, std::size_t count = 300, unsigned int seed = 1337);

    void render(sf::RenderTarget& target) const;
    std::size_t star_count() const;
    const std::vector<sf::Vector2f>& positions() const;

private:
    std::vector<sf::Vector2f> positions_;
    std::vector<sf::CircleShape> shapes_;
};
