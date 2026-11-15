#pragma once

#include "integrator.hpp"

/**
 * @file rk4_integrator.hpp
 * @brief Classical fourth-order Runge-Kutta (RK4) numerical integrator.
 */

/**
 * @class RK4Integrator
 * @brief Implements classical 4th-order Runge-Kutta numerical integration for coupled second-order ODEs.
 *
 * For state vector \f$ \mathbf{y} = [\mathbf{x}, \mathbf{v}]^T \f$ where
 * \f$ \frac{d\mathbf{y}}{dt} = [\mathbf{v}, \mathbf{a}(\mathbf{x})]^T \f$:
 *
 * \f[
 * \mathbf{k}_1 = f(t_n, \mathbf{y}_n)
 * \f]
 * \f[
 * \mathbf{k}_2 = f(t_n + \frac{\Delta t}{2}, \mathbf{y}_n + \frac{\Delta t}{2} \mathbf{k}_1)
 * \f]
 * \f[
 * \mathbf{k}_3 = f(t_n + \frac{\Delta t}{2}, \mathbf{y}_n + \frac{\Delta t}{2} \mathbf{k}_2)
 * \f]
 * \f[
 * \mathbf{k}_4 = f(t_n + \Delta t, \mathbf{y}_n + \Delta t \, \mathbf{k}_3)
 * \f]
 * \f[
 * \mathbf{y}_{n+1} = \mathbf{y}_n + \frac{\Delta t}{6} (\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
 * \f]
 *
 * @note Provides fourth-order convergence (\f$O(\Delta t^4)\f$), preserving stable orbits
 * with significantly lower phase and energy error than first-order methods.
 */
class RK4Integrator : public IIntegrator {
public:
    /**
     * @brief Performs one 4-stage RK4 integration step over all bodies.
     *
     * @param bodies Collection of bodies updated in-place.
     * @param dt Integration timestep.
     * @param accel_fn Callback evaluating accelerations at given intermediate positions.
     */
    void step(std::vector<Body>& bodies, float dt,
              const AccelerationFn& accel_fn) override;
};
