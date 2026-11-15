#pragma once

#include "body.hpp"
#include "integrator.hpp"
#include "sim_config.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <SFML/System/Vector2.hpp>

class PhysicsWorld {
public:
    explicit PhysicsWorld(SimConfig config);
    void set_bodies(std::vector<Body> bodies);
    void step();  // one fixed-timestep physics step
    const std::vector<Body>& bodies() const;
    std::size_t body_count() const;
    void set_integrator(const std::string& name);
    const std::string& integrator_name() const;
    void set_dt(float dt);
    float dt() const;

private:
    SimConfig config_;
    std::vector<Body> bodies_;
    std::unique_ptr<IIntegrator> integrator_;
    std::vector<sf::Vector2f> compute_accelerations(
        const std::vector<sf::Vector2f>& positions,
        const std::vector<float>& masses) const;
    void resolve_collisions();
};
