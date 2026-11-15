#include "sim_presets.hpp"
#include <cmath>
#include <random>

/**
 * @file sim_presets.cpp
 * @brief Implementations of simulation preset generator functions.
 */

namespace presets {

const std::vector<ColorRGB> kNeonPalette = {
    {255, 107, 0},   // Neon Orange
    {0, 210, 255},   // Electric Blue
    {57, 255, 20},   // Neon Green
    {255, 23, 68},   // Neon Red
    {255, 255, 0},   // Cyber / Neon Yellow
    {180, 0, 255},   // Neon Purple / Violet
    {255, 0, 128},   // Hot Pink / Magenta
    {0, 255, 230},   // Bright Cyan / Aqua
    {204, 255, 0},   // Bright Lime
    {255, 170, 0}    // Neon Amber
};

std::vector<Body> create_stable_orbit(const SimConfig& cfg) {
    std::vector<Body> bodies;
    bodies.reserve(2);

    // Place central body at physical center of the window
    const sf::Vector2f center(
        static_cast<float>(cfg.window_width) * 0.5f,
        static_cast<float>(cfg.window_height) * 0.5f
    );

    const float central_mass = 10000.0f;
    const float central_radius = 15.0f;

    const float orbiter_mass = 1.0f;
    const float orbiter_radius = 4.0f;
    const float distance = 200.0f;

    // Circular orbit speed balancing gravitational acceleration: v = sqrt(G * M / d)
    const float speed = std::sqrt(cfg.G * central_mass / distance);

    // Central body at center of physics space with zero initial velocity
    bodies.push_back(Body{
        1,
        central_mass,
        center,
        sf::Vector2f(0.0f, 0.0f),
        central_radius
    });

    // Orbiter at (center.x + distance, center.y) with perpendicular tangential velocity
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

    // Central sun position
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

    // Planetary definitions with exponential AU distance ratios and realistic radii:
    // Mercury (0.39 AU), Venus (0.72 AU), Earth (1.00 AU), Mars (1.52 AU), Jupiter (5.20 AU scaled)
    struct PlanetDef {
        float distance;
        float mass;
        float radius;
    };

    const PlanetDef planets[] = {
        {  50.0f, 0.5f, 2.5f}, // Mercury
        {  90.0f, 1.0f, 3.5f}, // Venus
        { 125.0f, 1.0f, 4.0f}, // Earth
        { 190.0f, 0.7f, 3.0f}, // Mars
        { 340.0f, 8.0f, 8.0f}  // Jupiter
    };

    uint64_t next_id = 2;
    for (const auto& p : planets) {
        // Orbital speed for circular trajectory around the central sun
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

    // Inset padding (5%) to prevent immediate border clipping
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
    std::uniform_int_distribution<size_t> dist_color(0, kNeonPalette.size() - 1);

    for (size_t i = 0; i < num_particles; ++i) {
        const uint64_t id = static_cast<uint64_t>(i + 1);
        const float mass = dist_m(gen);
        const sf::Vector2f pos{dist_x(gen), dist_y(gen)};
        const sf::Vector2f vel{dist_v(gen), dist_v(gen)};
        // Volume-proportional radius assuming constant material density: r ~ m^(1/3)
        const float radius = 2.0f * std::cbrt(mass);
        const ColorRGB color = kNeonPalette[dist_color(gen)];

        bodies.push_back(Body{id, mass, pos, vel, radius, color});
    }

    return bodies;
}

} // namespace presets
