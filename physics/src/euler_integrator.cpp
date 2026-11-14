#include "euler_integrator.hpp"

#include <cassert>
#include <cstddef>
#include <vector>

void EulerIntegrator::step(std::vector<Body>& bodies, float dt,
                           const AccelerationFn& accel_fn) {
    std::vector<sf::Vector2f> positions;
    positions.reserve(bodies.size());
    std::vector<float> masses;
    masses.reserve(bodies.size());

    for (const auto& body : bodies) {
        positions.push_back(body.position);
        masses.push_back(body.mass);
    }

    std::vector<sf::Vector2f> accelerations = accel_fn(positions, masses);
    assert(accelerations.size() == bodies.size());

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        sf::Vector2f old_v = bodies[i].velocity;
        bodies[i].velocity += dt * accelerations[i];
        bodies[i].position += dt * old_v;
    }
}
