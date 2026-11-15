#pragma once

#include "body.hpp"
#include "integrator.hpp"
#include "sim_config.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <SFML/System/Vector2.hpp>

/**
 * @file physics_world.hpp
 * @brief Encapsulates the N-body gravitational simulation world, particles, and collision pipeline.
 */

/**
 * @class PhysicsWorld
 * @brief Manages gravitational particle state, pairwise force computations, numerical integration, and collision resolution.
 */
class PhysicsWorld {
public:
    /**
     * @brief Constructs a PhysicsWorld with given configuration parameters.
     * @param config Simulation configuration (G, epsilon, dt, integrator selection).
     */
    explicit PhysicsWorld(SimConfig config);

    /**
     * @brief Sets or replaces the current collection of celestial bodies.
     * @param bodies New vector of Body structs.
     */
    void set_bodies(std::vector<Body> bodies);

    /**
     * @brief Executes one fixed-timestep physics update:
     * 1. Integrates positions and velocities using the active integrator over timestep dt.
     * 2. Resolves overlapping particle collisions via completely inelastic coalescing merges.
     */
    void step();

    /**
     * @brief Gets an immutable reference to the collection of simulated bodies.
     * @return const std::vector<Body>& Vector of active bodies.
     */
    const std::vector<Body>& bodies() const;

    /**
     * @brief Returns the total number of active celestial bodies.
     * @return std::size_t Body count.
     */
    std::size_t body_count() const;

    /**
     * @brief Swaps the active numerical integrator at runtime without disturbing particle state.
     * @param name Integrator algorithm name ("euler" or "rk4").
     */
    void set_integrator(const std::string& name);

    /**
     * @brief Returns the name of the currently active numerical integrator.
     * @return const std::string& "euler" or "rk4".
     */
    const std::string& integrator_name() const;

    /**
     * @brief Updates the integration timestep dt at runtime.
     * @param dt New integration timestep in seconds.
     */
    void set_dt(float dt);

    /**
     * @brief Returns the current integration timestep dt.
     * @return float Timestep in seconds.
     */
    float dt() const;

private:
    SimConfig config_;                          ///< Active simulation configuration.
    std::vector<Body> bodies_;                  ///< Managed celestial bodies.
    std::unique_ptr<IIntegrator> integrator_;   ///< Polymorphic numerical integrator instance.

    /**
     * @brief Computes pairwise Newtonian gravitational accelerations with Plummer softening.
     *
     * @param positions Coordinate positions of all bodies.
     * @param masses Mass values of all bodies.
     * @return std::vector<sf::Vector2f> Net acceleration vector for each body.
     */
    std::vector<sf::Vector2f> compute_accelerations(
        const std::vector<sf::Vector2f>& positions,
        const std::vector<float>& masses) const;

    /**
     * @brief Detects and resolves geometric overlaps between bodies through inelastic merging.
     */
    void resolve_collisions();
};
