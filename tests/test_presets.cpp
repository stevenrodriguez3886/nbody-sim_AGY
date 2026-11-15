#include "sim_presets.hpp"
#include "sim_config.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

void test_stable_orbit() {
    SimConfig cfg;
    cfg.window_width = 1280;
    cfg.window_height = 720;
    cfg.G = 2.0f;

    auto bodies = presets::create_stable_orbit(cfg);
    assert(bodies.size() == 2);

    const auto& sun = bodies[0];
    const auto& planet = bodies[1];

    // Central body should be much more massive
    assert(sun.mass > planet.mass);
    assert(sun.mass >= 1000.0f);
    assert(planet.mass > 0.0f);

    // Central body should be at center of window with 0 velocity
    sf::Vector2f center(cfg.window_width * 0.5f, cfg.window_height * 0.5f);
    assert(std::abs(sun.position.x - center.x) < 1e-3f);
    assert(std::abs(sun.position.y - center.y) < 1e-3f);
    assert(std::abs(sun.velocity.x) < 1e-3f);
    assert(std::abs(sun.velocity.y) < 1e-3f);

    // Orbiter should be at distance d along x axis
    float dx = planet.position.x - sun.position.x;
    float dy = planet.position.y - sun.position.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    assert(dist > 50.0f);

    // Velocity should be perpendicular and equal to sqrt(G * M / dist)
    float expected_v = std::sqrt(cfg.G * sun.mass / dist);
    float speed = std::sqrt(planet.velocity.x * planet.velocity.x + planet.velocity.y * planet.velocity.y);
    assert(std::abs(speed - expected_v) < 1e-2f);

    // Dot product of (r - r_sun) and v should be ~0 (circular orbit)
    float dot = dx * planet.velocity.x + dy * planet.velocity.y;
    assert(std::abs(dot) < 1e-3f);

    // Radii should be positive
    assert(sun.radius > 0.0f);
    assert(planet.radius > 0.0f);

    // Unique IDs
    assert(sun.id != planet.id);
}

void test_solar_system() {
    SimConfig cfg;
    cfg.window_width = 1280;
    cfg.window_height = 720;
    cfg.G = 1.0f;

    auto bodies = presets::create_solar_system(cfg);
    assert(bodies.size() == 6); // 1 sun + 5 planets: Mercury, Venus, Earth, Mars, Jupiter

    const auto& sun = bodies[0];
    sf::Vector2f center(cfg.window_width * 0.5f, cfg.window_height * 0.5f);
    assert(std::abs(sun.position.x - center.x) < 1e-3f);
    assert(std::abs(sun.position.y - center.y) < 1e-3f);

    float prev_dist = 0.0f;
    std::set<uint64_t> ids;
    ids.insert(sun.id);
    std::vector<float> dists;

    for (size_t i = 1; i < bodies.size(); ++i) {
        const auto& p = bodies[i];
        assert(ids.find(p.id) == ids.end()); // unique id
        ids.insert(p.id);

        assert(sun.mass > p.mass);

        float dx = p.position.x - sun.position.x;
        float dy = p.position.y - sun.position.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Planets should be at increasing distances
        assert(dist > prev_dist);
        prev_dist = dist;
        dists.push_back(dist);

        // Circular velocity v = sqrt(G * M / dist)
        float expected_v = std::sqrt(cfg.G * sun.mass / dist);
        float speed = std::sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y);
        assert(std::abs(speed - expected_v) < 1e-2f);

        // Perpendicular velocity
        float dot = dx * p.velocity.x + dy * p.velocity.y;
        assert(std::abs(dot) < 1e-3f);

        assert(p.radius > 0.0f);
    }

    // Increasing orbital gap jumps: outer jump d5 - d4 > 100.0f while inner gap d2 - d1 < 60.0f
    assert((dists[4] - dists[3]) > 100.0f);
    assert((dists[1] - dists[0]) < 60.0f);

    // Distinct radii: Jupiter radius > Earth radius > Mercury radius
    assert(bodies[5].radius > bodies[3].radius);
    assert(bodies[3].radius > bodies[1].radius);
}

void test_random_cloud() {
    SimConfig cfg;
    cfg.window_width = 1280;
    cfg.window_height = 720;

    auto cloud1 = presets::create_random_cloud(cfg, 42);
    auto cloud2 = presets::create_random_cloud(cfg, 42);

    // Deterministic: identical size and values
    assert(cloud1.size() >= 50 && cloud1.size() <= 100);
    assert(cloud1.size() == cloud2.size());

    std::set<uint64_t> ids;
    for (size_t i = 0; i < cloud1.size(); ++i) {
        const auto& a = cloud1[i];
        const auto& b = cloud2[i];

        assert(a.id == b.id);
        assert(ids.find(a.id) == ids.end());
        ids.insert(a.id);

        assert(a.mass == b.mass);
        assert(a.mass >= 1.0f && a.mass <= 10.0f);

        assert(a.position.x == b.position.x);
        assert(a.position.y == b.position.y);
        // Positions within bounds
        assert(a.position.x >= 0.0f && a.position.x <= static_cast<float>(cfg.window_width));
        assert(a.position.y >= 0.0f && a.position.y <= static_cast<float>(cfg.window_height));

        assert(a.velocity.x == b.velocity.x);
        assert(a.velocity.y == b.velocity.y);

        assert(a.radius == b.radius);
        assert(a.radius > 0.0f);
    }

    // Different seed produces different results
    auto cloud_diff = presets::create_random_cloud(cfg, 999);
    bool any_difference = (cloud1.size() != cloud_diff.size());
    if (!any_difference) {
        for (size_t i = 0; i < cloud1.size(); ++i) {
            if (cloud1[i].position.x != cloud_diff[i].position.x ||
                cloud1[i].position.y != cloud_diff[i].position.y) {
                any_difference = true;
                break;
            }
        }
    }
    assert(any_difference);
}

void test_presets_respect_config() {
    SimConfig cfg1;
    cfg1.window_width = 800;
    cfg1.window_height = 600;
    cfg1.G = 1.0f;

    SimConfig cfg2;
    cfg2.window_width = 1600;
    cfg2.window_height = 1200;
    cfg2.G = 4.0f;

    auto orbit1 = presets::create_stable_orbit(cfg1);
    auto orbit2 = presets::create_stable_orbit(cfg2);

    // Center shifts with window dimensions
    assert(std::abs(orbit1[0].position.x - 400.0f) < 1e-3f);
    assert(std::abs(orbit1[0].position.y - 300.0f) < 1e-3f);
    assert(std::abs(orbit2[0].position.x - 800.0f) < 1e-3f);
    assert(std::abs(orbit2[0].position.y - 600.0f) < 1e-3f);

    // G factor of 4 increases circular velocity by factor of 2 (sqrt(4) = 2)
    float speed1 = std::sqrt(orbit1[1].velocity.x * orbit1[1].velocity.x + orbit1[1].velocity.y * orbit1[1].velocity.y);
    float speed2 = std::sqrt(orbit2[1].velocity.x * orbit2[1].velocity.x + orbit2[1].velocity.y * orbit2[1].velocity.y);
    assert(std::abs(speed2 - 2.0f * speed1) < 1e-2f);
}

int main() {
    test_stable_orbit();
    test_solar_system();
    test_random_cloud();
    test_presets_respect_config();
    std::cout << "All presets tests passed" << std::endl;
    return 0;
}
