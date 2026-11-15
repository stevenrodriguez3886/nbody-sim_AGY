#pragma once

#include "integrator.hpp"

/**
 * @file euler_integrator.hpp
 * @brief First-order explicit Euler numerical integrator.
 */

/**
 * @class EulerIntegrator
 * @brief Implements explicit (forward) Euler numerical integration:
 *
 * \f[
 * \mathbf{x}_{n+1} = \mathbf{x}_n + \Delta t \, \mathbf{v}_n
 * \f]
 * \f[
 * \mathbf{v}_{n+1} = \mathbf{v}_n + \Delta t \, \mathbf{a}_n
 * \f]
 *
 * @note First-order accuracy (\f$O(\Delta t)\f$). Fast and simple, but non-symplectic
 * and exhibits energy drift over long orbital trajectories.
 */
class EulerIntegrator : public IIntegrator {
public:
    /**
     * @brief Performs one forward Euler step over all bodies.
     *
     * @param bodies Collection of bodies updated in-place.
     * @param dt Integration timestep.
     * @param accel_fn Callback evaluating accelerations at current positions.
     */
    void step(std::vector<Body>& bodies, float dt,
              const AccelerationFn& accel_fn) override;
};
