#include "rk4_integrator.hpp"

#include <cassert>
#include <cstddef>
#include <vector>

/**
 * @file rk4_integrator.cpp
 * @brief Implementation of classical 4th-order Runge-Kutta (RK4) time integration.
 */

void RK4Integrator::step(std::vector<Body>& bodies, float dt,
                         const AccelerationFn& accel_fn) {
    if (bodies.empty()) {
        return;
    }

    const std::size_t n = bodies.size();

    // Extract initial state vectors: position x_0 and velocity v_0
    std::vector<sf::Vector2f> x(n);
    std::vector<sf::Vector2f> v(n);
    std::vector<float> masses(n);

    for (std::size_t i = 0; i < n; ++i) {
        x[i] = bodies[i].position;
        v[i] = bodies[i].velocity;
        masses[i] = bodies[i].mass;
    }

    // =========================================================================
    // Stage 1: Evaluate derivative at initial point (t_n, y_n)
    // k1_x = v
    // k1_v = a(x)
    // =========================================================================
    std::vector<sf::Vector2f> a1 = accel_fn(x, masses);
    assert(a1.size() == n);

    std::vector<sf::Vector2f> k1_x(n);
    std::vector<sf::Vector2f> k1_v(n);
    for (std::size_t i = 0; i < n; ++i) {
        k1_x[i] = v[i];
        k1_v[i] = a1[i];
    }

    // =========================================================================
    // Stage 2: Evaluate derivative at midpoint using Euler trial step with k1
    // x2 = x + 0.5 * dt * k1_x
    // v2 = v + 0.5 * dt * k1_v
    // k2_x = v2
    // k2_v = a(x2)
    // =========================================================================
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

    // =========================================================================
    // Stage 3: Evaluate derivative at midpoint using Euler trial step with k2
    // x3 = x + 0.5 * dt * k2_x
    // v3 = v + 0.5 * dt * k2_v
    // k3_x = v3
    // k3_v = a(x3)
    // =========================================================================
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

    // =========================================================================
    // Stage 4: Evaluate derivative at endpoint (t_n + dt) using full step with k3
    // x4 = x + dt * k3_x
    // v4 = v + dt * k3_v
    // k4_x = v4
    // k4_v = a(x4)
    // =========================================================================
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

    // =========================================================================
    // Final weighted integration update (Simpson's 1/3 rule weights: 1, 2, 2, 1)
    // y_{n+1} = y_n + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
    // =========================================================================
    const float dt6 = dt / 6.0f;
    for (std::size_t i = 0; i < n; ++i) {
        bodies[i].position += dt6 * (k1_x[i] + 2.0f * k2_x[i] + 2.0f * k3_x[i] + k4_x[i]);
        bodies[i].velocity += dt6 * (k1_v[i] + 2.0f * k2_v[i] + 2.0f * k3_v[i] + k4_v[i]);
    }
}
