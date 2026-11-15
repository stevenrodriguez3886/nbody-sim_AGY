#include "body.hpp"

#include <algorithm>
#include <cmath>

/**
 * @file body.cpp
 * @brief Implementation of body merging and collision detection routines.
 */

Body merge(const Body& a, const Body& b) {
    const float total_mass = a.mass + b.mass;
    sf::Vector2f new_position;
    sf::Vector2f new_velocity;

    // Weight position and velocity by mass (center of mass and conservation of linear momentum)
    if (total_mass > 0.0f) {
        new_position = (a.mass * a.position + b.mass * b.position) / total_mass;
        new_velocity = (a.mass * a.velocity + b.mass * b.velocity) / total_mass;
    } else {
        // Fallback for massless particles: geometric midpoint and mean velocity
        new_position = 0.5f * (a.position + b.position);
        new_velocity = 0.5f * (a.velocity + b.velocity);
    }

    // Preserve 2D disc area: pi * r_new^2 = pi * r_a^2 + pi * r_b^2 -> r_new = sqrt(r_a^2 + r_b^2)
    const float new_radius = std::sqrt(a.radius * a.radius + b.radius * b.radius);

    // Retain ID of the dominant progenitor body; break ties deterministically with smaller ID
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
    const float dx = b.position.x - a.position.x;
    const float dy = b.position.y - a.position.y;
    // std::hypot safely calculates Euclidean distance avoiding intermediate overflow/underflow
    const float dist = std::hypot(dx, dy);
    return dist < (a.radius + b.radius);
}
