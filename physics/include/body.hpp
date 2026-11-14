#pragma once

#include <cstdint>
#include <SFML/System/Vector2.hpp>

struct Body {
    uint64_t id;
    float mass;
    sf::Vector2f position;
    sf::Vector2f velocity;
    float radius;
};

Body merge(const Body& a, const Body& b);
bool check_collision(const Body& a, const Body& b);
