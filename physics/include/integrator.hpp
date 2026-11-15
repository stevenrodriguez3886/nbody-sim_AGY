#pragma once

#include "body.hpp"
#include <functional>
#include <vector>
#include <SFML/System/Vector2.hpp>

/**
 * @file integrator.hpp
 * @brief Polymorphic interface for numerical ODE integrators in the N-body simulation.
 */

/**
 * @typedef AccelerationFn
 * @brief Function callback for evaluating pairwise gravitational accelerations.
 *
 * Given a vector of positions and corresponding masses, returns a vector of
 * acceleration vectors for each body: \f$ \mathbf{a}_i(\mathbf{x}_1, \dots, \mathbf{x}_N) \f$.
 */
using AccelerationFn = std::function<std::vector<sf::Vector2f>(
    const std::vector<sf::Vector2f>& positions,
    const std::vector<float>& masses)>;

/**
 * @class IIntegrator
 * @brief Abstract base class defining the numerical time-integration interface.
 */
class IIntegrator {
public:
    virtual ~IIntegrator() = default;

    /**
     * @brief Advances the positions and velocities of a system of bodies by time step dt.
     *
     * @param bodies Collection of bodies whose positions and velocities are updated in-place.
     * @param dt Time delta for the integration step (seconds).
     * @param accel_fn Callback providing acceleration evaluations for given state positions.
     */
    virtual void step(std::vector<Body>& bodies, float dt,
                      const AccelerationFn& accel_fn) = 0;
};
