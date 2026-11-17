#include "body.hpp"

#include <algorithm>
#include <cmath>

/**
 * @file body.cpp
 * @brief Implementation of elastic collision resolution and collision detection routines.
 */

void resolve_elastic_collision(Body& a, Body& b) {
    const float dx = b.position.x - a.position.x;
    const float dy = b.position.y - a.position.y;
    const float dist = std::hypot(dx, dy);
    const float min_dist = a.radius + b.radius;

    // Strict inequality: only resolve if bodies overlap
    if (dist >= min_dist) {
        return;
    }

    // Determine normal vector from a to b; handle degenerate zero-distance case
    sf::Vector2f normal(1.0f, 0.0f);
    if (dist > 1e-6f) {
        normal.x = dx / dist;
        normal.y = dy / dist;
    }

    // Elastic velocity impulse resolution (along collision normal)
    const sf::Vector2f rel_vel = b.velocity - a.velocity;
    const float v_normal = rel_vel.x * normal.x + rel_vel.y * normal.y;

    // If bodies are already separating, do not apply a repulsive impulse
    if (v_normal >= 0.0f) {
        return;
    }

    const float total_mass = a.mass + b.mass;
    float factor_a = 1.0f;
    float factor_b = 1.0f;
    if (total_mass > 0.0f) {
        factor_a = 2.0f * b.mass / total_mass;
        factor_b = 2.0f * a.mass / total_mass;
    }

    a.velocity += factor_a * v_normal * normal;
    b.velocity -= factor_b * v_normal * normal;
}

bool check_collision(const Body& a, const Body& b) {
    const float dx = b.position.x - a.position.x;
    const float dy = b.position.y - a.position.y;
    // std::hypot safely calculates Euclidean distance avoiding intermediate overflow/underflow
    const float dist = std::hypot(dx, dy);
    return dist < (a.radius + b.radius);
}
