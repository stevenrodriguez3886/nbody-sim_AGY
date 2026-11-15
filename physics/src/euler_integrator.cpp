#include "euler_integrator.hpp"

#include <cassert>
#include <cstddef>
#include <vector>

/**
 * @file euler_integrator.cpp
 * @brief Implementation of explicit Euler time stepping.
 */

void EulerIntegrator::step(std::vector<Body>& bodies, float dt,
                           const AccelerationFn& accel_fn) {
    const std::size_t n = bodies.size();

    // Extract current state vectors for acceleration evaluation
    std::vector<sf::Vector2f> positions;
    positions.reserve(n);
    std::vector<float> masses;
    masses.reserve(n);

    for (const auto& body : bodies) {
        positions.push_back(body.position);
        masses.push_back(body.mass);
    }

    // Compute pairwise accelerations at time t_n
    std::vector<sf::Vector2f> accelerations = accel_fn(positions, masses);
    assert(accelerations.size() == n);

    // Explicit Euler update:
    // v_{n+1} = v_n + dt * a_n
    // x_{n+1} = x_n + dt * v_n (using pre-update velocity old_v)
    for (std::size_t i = 0; i < n; ++i) {
        const sf::Vector2f old_v = bodies[i].velocity;
        bodies[i].velocity += dt * accelerations[i];
        bodies[i].position += dt * old_v;
    }
}
