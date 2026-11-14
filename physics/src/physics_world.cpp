#include "physics_world.hpp"
#include "euler_integrator.hpp"
#include "rk4_integrator.hpp"

#include <cmath>
#include <utility>

PhysicsWorld::PhysicsWorld(SimConfig config)
    : config_(std::move(config)) {
    if (config_.integrator == "rk4") {
        integrator_ = std::make_unique<RK4Integrator>();
    } else {
        integrator_ = std::make_unique<EulerIntegrator>();
    }
}

void PhysicsWorld::set_bodies(std::vector<Body> bodies) {
    bodies_ = std::move(bodies);
}

void PhysicsWorld::step() {
    integrator_->step(bodies_, config_.dt, [this](const auto& positions, const auto& masses) {
        return compute_accelerations(positions, masses);
    });
    resolve_collisions();
}

const std::vector<Body>& PhysicsWorld::bodies() const {
    return bodies_;
}

std::size_t PhysicsWorld::body_count() const {
    return bodies_.size();
}

std::vector<sf::Vector2f> PhysicsWorld::compute_accelerations(
    const std::vector<sf::Vector2f>& positions,
    const std::vector<float>& masses) const {
    const std::size_t n = positions.size();
    std::vector<sf::Vector2f> accelerations(n, sf::Vector2f{0.f, 0.f});

    if (n <= 1) {
        return accelerations;
    }

    const float eps2 = config_.epsilon * config_.epsilon;

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            sf::Vector2f r_ij = positions[j] - positions[i];
            float r2 = r_ij.x * r_ij.x + r_ij.y * r_ij.y;
            float dist_sq = r2 + eps2;
            float denom = dist_sq * std::sqrt(dist_sq);
            if (denom > 0.0f) {
                float factor = (config_.G * masses[j]) / denom;
                accelerations[i].x += factor * r_ij.x;
                accelerations[i].y += factor * r_ij.y;
            }
        }
    }

    return accelerations;
}

void PhysicsWorld::resolve_collisions() {
    bool merged = true;
    while (merged) {
        merged = false;
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            for (std::size_t j = i + 1; j < bodies_.size(); ++j) {
                if (check_collision(bodies_[i], bodies_[j])) {
                    bodies_[i] = merge(bodies_[i], bodies_[j]);
                    bodies_.erase(bodies_.begin() + j);
                    merged = true;
                    break;
                }
            }
            if (merged) {
                break;
            }
        }
    }
}
