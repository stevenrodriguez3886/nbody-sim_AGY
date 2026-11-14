#pragma once

#include "body.hpp"
#include "integrator.hpp"
#include "sim_config.hpp"

#include <cstddef>
#include <memory>
#include <vector>
#include <SFML/System/Vector2.hpp>

class PhysicsWorld {
public:
    explicit PhysicsWorld(SimConfig config);
    void set_bodies(std::vector<Body> bodies);
    void step();  // one fixed-timestep physics step
    const std::vector<Body>& bodies() const;
    std::size_t body_count() const;

private:
    SimConfig config_;
    std::vector<Body> bodies_;
    std::unique_ptr<IIntegrator> integrator_;
    std::vector<sf::Vector2f> compute_accelerations(
        const std::vector<sf::Vector2f>& positions,
        const std::vector<float>& masses) const;
    void resolve_collisions();
};
