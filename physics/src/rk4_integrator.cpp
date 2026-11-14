#include "rk4_integrator.hpp"

#include <cassert>
#include <cstddef>
#include <vector>

void RK4Integrator::step(std::vector<Body>& bodies, float dt,
                         const AccelerationFn& accel_fn) {
    if (bodies.empty()) {
        return;
    }

    const std::size_t n = bodies.size();

    std::vector<sf::Vector2f> x(n);
    std::vector<sf::Vector2f> v(n);
    std::vector<float> masses(n);

    for (std::size_t i = 0; i < n; ++i) {
        x[i] = bodies[i].position;
        v[i] = bodies[i].velocity;
        masses[i] = bodies[i].mass;
    }

    // Stage 1
    std::vector<sf::Vector2f> a1 = accel_fn(x, masses);
    assert(a1.size() == n);

    std::vector<sf::Vector2f> k1_x(n);
    std::vector<sf::Vector2f> k1_v(n);
    for (std::size_t i = 0; i < n; ++i) {
        k1_x[i] = v[i];
        k1_v[i] = a1[i];
    }

    // Stage 2
    std::vector<sf::Vector2f> x2(n);
    for (std::size_t i = 0; i < n; ++i) {
        x2[i] = x[i] + 0.5f * dt * k1_x[i];
    }
    std::vector<sf::Vector2f> a2 = accel_fn(x2, masses);
    assert(a2.size() == n);

    std::vector<sf::Vector2f> k2_x(n);
    std::vector<sf::Vector2f> k2_v(n);
    for (std::size_t i = 0; i < n; ++i) {
        k2_x[i] = v[i] + 0.5f * dt * k1_v[i];
        k2_v[i] = a2[i];
    }

    // Stage 3
    std::vector<sf::Vector2f> x3(n);
    for (std::size_t i = 0; i < n; ++i) {
        x3[i] = x[i] + 0.5f * dt * k2_x[i];
    }
    std::vector<sf::Vector2f> a3 = accel_fn(x3, masses);
    assert(a3.size() == n);

    std::vector<sf::Vector2f> k3_x(n);
    std::vector<sf::Vector2f> k3_v(n);
    for (std::size_t i = 0; i < n; ++i) {
        k3_x[i] = v[i] + 0.5f * dt * k2_v[i];
        k3_v[i] = a3[i];
    }

    // Stage 4
    std::vector<sf::Vector2f> x4(n);
    for (std::size_t i = 0; i < n; ++i) {
        x4[i] = x[i] + dt * k3_x[i];
    }
    std::vector<sf::Vector2f> a4 = accel_fn(x4, masses);
    assert(a4.size() == n);

    std::vector<sf::Vector2f> k4_x(n);
    std::vector<sf::Vector2f> k4_v(n);
    for (std::size_t i = 0; i < n; ++i) {
        k4_x[i] = v[i] + dt * k3_v[i];
        k4_v[i] = a4[i];
    }

    // Final update
    const float dt6 = dt / 6.0f;
    for (std::size_t i = 0; i < n; ++i) {
        bodies[i].position += dt6 * (k1_x[i] + 2.0f * k2_x[i] + 2.0f * k3_x[i] + k4_x[i]);
        bodies[i].velocity += dt6 * (k1_v[i] + 2.0f * k2_v[i] + 2.0f * k3_v[i] + k4_v[i]);
    }
}
