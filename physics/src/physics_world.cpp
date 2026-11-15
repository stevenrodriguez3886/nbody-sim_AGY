#include "physics_world.hpp"
#include "euler_integrator.hpp"
#include "rk4_integrator.hpp"

#include <cmath>
#include <utility>

/**
 * @file physics_world.cpp
 * @brief Implementation of physics world state, gravitational forces, and collision handling.
 */

PhysicsWorld::PhysicsWorld(SimConfig config)
    : config_(std::move(config)) {
    // Instantiate selected integrator based on initial config
    set_integrator(config_.integrator);
}

void PhysicsWorld::set_bodies(std::vector<Body> bodies) {
    bodies_ = std::move(bodies);
}

void PhysicsWorld::set_integrator(const std::string& name) {
    config_.integrator = name;
    if (config_.integrator == "rk4") {
        integrator_ = std::make_unique<RK4Integrator>();
    } else {
        integrator_ = std::make_unique<EulerIntegrator>();
    }
}

const std::string& PhysicsWorld::integrator_name() const {
    return config_.integrator;
}

void PhysicsWorld::set_dt(float dt) {
    config_.dt = dt;
}

float PhysicsWorld::dt() const {
    return config_.dt;
}

void PhysicsWorld::step() {
    // 1. Advance state variables using the polymorphic integrator
    integrator_->step(bodies_, config_.dt, [this](const auto& positions, const auto& masses) {
        return compute_accelerations(positions, masses);
    });

    // 2. Resolve any overlapping collisions resulting from the new positions
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

    // Plummer softening length squared: avoids singular division by zero when particles get close
    const float eps2 = config_.epsilon * config_.epsilon;

    // Pairwise N-body Newtonian gravity:
    // a_i = sum_{j != i} G * m_j * (r_j - r_i) / (||r_j - r_i||^2 + eps^2)^(3/2)
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            const sf::Vector2f r_ij = positions[j] - positions[i];
            const float r2 = r_ij.x * r_ij.x + r_ij.y * r_ij.y;
            const float dist_sq = r2 + eps2;
            const float denom = dist_sq * std::sqrt(dist_sq); // (r^2 + eps^2)^(3/2)
            if (denom > 0.0f) {
                const float factor = (config_.G * masses[j]) / denom;
                accelerations[i].x += factor * r_ij.x;
                accelerations[i].y += factor * r_ij.y;
            }
        }
    }

    return accelerations;
}

void PhysicsWorld::resolve_collisions() {
    // Greedy pairwise collision coalescence:
    // When two bodies overlap, merge them into body i and remove body j.
    // Repeat until no overlapping pairs remain.
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

double PhysicsWorld::compute_kinetic_energy() const {
    double total_ek = 0.0;
    for (const auto& body : bodies_) {
        const double vx = static_cast<double>(body.velocity.x);
        const double vy = static_cast<double>(body.velocity.y);
        const double v2 = vx * vx + vy * vy;
        total_ek += 0.5 * static_cast<double>(body.mass) * v2;
    }
    return total_ek;
}

double PhysicsWorld::compute_potential_energy() const {
    const std::size_t n = bodies_.size();
    if (n <= 1) {
        return 0.0;
    }

    const double eps2 = static_cast<double>(config_.epsilon) * static_cast<double>(config_.epsilon);
    const double G = static_cast<double>(config_.G);
    double total_ep = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = static_cast<double>(bodies_[j].position.x - bodies_[i].position.x);
            const double dy = static_cast<double>(bodies_[j].position.y - bodies_[i].position.y);
            const double dist_sq = dx * dx + dy * dy + eps2;
            const double dist = std::sqrt(dist_sq);
            if (dist > 0.0) {
                total_ep -= (G * static_cast<double>(bodies_[i].mass) * static_cast<double>(bodies_[j].mass)) / dist;
            }
        }
    }
    return total_ep;
}

double PhysicsWorld::compute_total_energy() const {
    return compute_kinetic_energy() + compute_potential_energy();
}
