#include "sim_presets.hpp"
#include <cmath>
#include <random>

namespace presets {

std::vector<Body> create_stable_orbit(const SimConfig& cfg) {
    std::vector<Body> bodies;
    bodies.reserve(2);

    const sf::Vector2f center(
        static_cast<float>(cfg.window_width) * 0.5f,
        static_cast<float>(cfg.window_height) * 0.5f
    );

    const float central_mass = 10000.0f;
    const float central_radius = 15.0f;

    const float orbiter_mass = 1.0f;
    const float orbiter_radius = 4.0f;
    const float distance = 200.0f;

    // Circular orbit speed: v = sqrt(G * M / d)
    const float speed = std::sqrt(cfg.G * central_mass / distance);

    // Central body at center of physics space
    bodies.push_back(Body{
        1,
        central_mass,
        center,
        sf::Vector2f(0.0f, 0.0f),
        central_radius
    });

    // Orbiter at (center.x + distance, center.y) with perpendicular velocity
    bodies.push_back(Body{
        2,
        orbiter_mass,
        sf::Vector2f(center.x + distance, center.y),
        sf::Vector2f(0.0f, speed),
        orbiter_radius
    });

    return bodies;
}

std::vector<Body> create_solar_system(const SimConfig& cfg) {
    std::vector<Body> bodies;

    const sf::Vector2f center(
        static_cast<float>(cfg.window_width) * 0.5f,
        static_cast<float>(cfg.window_height) * 0.5f
    );

    const float sun_mass = 10000.0f;
    const float sun_radius = 15.0f;

    // Sun at center
    bodies.push_back(Body{
        1,
        sun_mass,
        center,
        sf::Vector2f(0.0f, 0.0f),
        sun_radius
    });

    struct PlanetDef {
        float distance;
        float mass;
        float radius;
    };

    const PlanetDef planets[] = {
        { 80.0f, 1.0f, 3.0f},
        {150.0f, 2.0f, 4.0f},
        {240.0f, 5.0f, 6.0f},
        {340.0f, 3.0f, 5.0f}
    };

    uint64_t next_id = 2;
    for (const auto& p : planets) {
        const float speed = std::sqrt(cfg.G * sun_mass / p.distance);
        bodies.push_back(Body{
            next_id++,
            p.mass,
            sf::Vector2f(center.x + p.distance, center.y),
            sf::Vector2f(0.0f, speed),
            p.radius
        });
    }

    return bodies;
}

std::vector<Body> create_random_cloud(const SimConfig& cfg, unsigned int seed) {
    std::mt19937 gen(seed);

    const size_t num_particles = 80;
    std::vector<Body> bodies;
    bodies.reserve(num_particles);

    float pad_x = static_cast<float>(cfg.window_width) * 0.05f;
    float max_x = static_cast<float>(cfg.window_width) - pad_x;
    float min_x = pad_x;
    if (max_x <= min_x) {
        min_x = 0.0f;
        max_x = static_cast<float>(cfg.window_width);
    }

    float pad_y = static_cast<float>(cfg.window_height) * 0.05f;
    float max_y = static_cast<float>(cfg.window_height) - pad_y;
    float min_y = pad_y;
    if (max_y <= min_y) {
        min_y = 0.0f;
        max_y = static_cast<float>(cfg.window_height);
    }

    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_v(-5.0f, 5.0f);
    std::uniform_real_distribution<float> dist_m(1.0f, 10.0f);

    for (size_t i = 0; i < num_particles; ++i) {
        const uint64_t id = static_cast<uint64_t>(i + 1);
        const float mass = dist_m(gen);
        const sf::Vector2f pos(dist_x(gen), dist_y(gen));
        const sf::Vector2f vel(dist_v(gen), dist_v(gen));
        const float radius = 2.0f * std::cbrt(mass);

        bodies.push_back(Body{id, mass, pos, vel, radius});
    }

    return bodies;
}

} // namespace presets
