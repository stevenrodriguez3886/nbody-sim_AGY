#include "body.hpp"

#include <algorithm>
#include <cmath>

Body merge(const Body& a, const Body& b) {
    float total_mass = a.mass + b.mass;
    sf::Vector2f new_position;
    sf::Vector2f new_velocity;

    if (total_mass > 0.0f) {
        new_position = (a.mass * a.position + b.mass * b.position) / total_mass;
        new_velocity = (a.mass * a.velocity + b.mass * b.velocity) / total_mass;
    } else {
        new_position = 0.5f * (a.position + b.position);
        new_velocity = 0.5f * (a.velocity + b.velocity);
    }

    float new_radius = std::sqrt(a.radius * a.radius + b.radius * b.radius);

    uint64_t new_id;
    if (a.mass > b.mass) {
        new_id = a.id;
    } else if (b.mass > a.mass) {
        new_id = b.id;
    } else {
        new_id = std::min(a.id, b.id);
    }

    return Body{new_id, total_mass, new_position, new_velocity, new_radius};
}

bool check_collision(const Body& a, const Body& b) {
    float dx = b.position.x - a.position.x;
    float dy = b.position.y - a.position.y;
    float dist = std::hypot(dx, dy);
    return dist < (a.radius + b.radius);
}
