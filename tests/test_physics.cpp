#include "physics_world.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

void test_two_body_attraction_euler() {
    // Two equal-mass bodies separated along x-axis, both initially at rest.
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    cfg.integrator = "euler";
    PhysicsWorld world(cfg);
    world.set_bodies({
        {1, 100.f, {0.f, 0.f}, {0.f, 0.f}, 1.f},
        {2, 100.f, {100.f, 0.f}, {0.f, 0.f}, 1.f}
    });

    // Step 1: In explicit Euler, acceleration updates velocity; position uses old velocity (0.0f)
    world.step();
    auto& b1 = world.bodies();
    assert(b1[0].velocity.x > 0.f);
    assert(b1[1].velocity.x < 0.f);

    // After multiple steps, accumulated displacement exceeds float precision ULP at x=100.f
    for (int i = 0; i < 20; ++i) {
        world.step();
    }
    auto& b2 = world.bodies();
    // Body 0 should have moved rightward (toward body 1)
    assert(b2[0].position.x > 0.f);
    // Body 1 should have moved leftward (toward body 0)
    assert(b2[1].position.x < 100.f);
}

void test_two_body_attraction_rk4() {
    SimConfig cfg;
    cfg.G = 10.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.05f;
    cfg.integrator = "rk4";
    PhysicsWorld world(cfg);
    world.set_bodies({
        {1, 100.f, {0.f, 0.f}, {0.f, 0.f}, 1.f},
        {2, 100.f, {10.f, 0.f}, {0.f, 0.f}, 1.f}
    });

    // RK4 intermediate stages update positions within a single step
    world.step();
    auto& b = world.bodies();
    assert(b[0].position.x > 0.f);
    assert(b[1].position.x < 10.f);
    assert(b[0].velocity.x > 0.f);
    assert(b[1].velocity.x < 0.f);
}

void test_collision_merge_reduces_count() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    cfg.integrator = "euler";
    PhysicsWorld world(cfg);
    // Two bodies overlapping -> should merge
    world.set_bodies({
        {1, 10.f, {0.f, 0.f}, {1.f, 0.f}, 5.f},
        {2, 10.f, {3.f, 0.f}, {-1.f, 0.f}, 5.f}  // distance 3 < 5+5
    });
    world.step();
    assert(world.body_count() == 1);
    assert(world.bodies()[0].mass == 20.f);
}

void test_collision_chain_merge() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    cfg.integrator = "euler";
    PhysicsWorld world(cfg);
    // Three bodies overlapping in a chain:
    // Body 1 at (0, 0) r=5, Body 2 at (4, 0) r=5, Body 3 at (8, 0) r=5
    world.set_bodies({
        {1, 10.f, {0.f, 0.f}, {0.f, 0.f}, 5.f},
        {2, 10.f, {4.f, 0.f}, {0.f, 0.f}, 5.f},
        {3, 10.f, {8.f, 0.f}, {0.f, 0.f}, 5.f}
    });
    world.step();
    assert(world.body_count() == 1);
    assert(world.bodies()[0].mass == 30.f);
}

void test_single_body_no_crash() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    cfg.integrator = "rk4";
    PhysicsWorld world(cfg);
    world.set_bodies({{1, 100.f, {50.f, 50.f}, {1.f, 0.f}, 5.f}});
    world.step();  // must not crash
    assert(world.body_count() == 1);
    // Body drifts at constant velocity (no forces)
    assert(std::abs(world.bodies()[0].position.x - 50.01f) < 0.1f);
}

void test_single_body_euler() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    cfg.integrator = "euler";
    PhysicsWorld world(cfg);
    world.set_bodies({{1, 100.f, {10.f, 20.f}, {2.f, -1.f}, 3.f}});
    world.step();
    assert(world.body_count() == 1);
    // Velocity remains unchanged (no forces)
    assert(std::abs(world.bodies()[0].velocity.x - 2.f) < 1e-5f);
    assert(std::abs(world.bodies()[0].velocity.y - (-1.f)) < 1e-5f);
}

void test_empty_world() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.01f;
    PhysicsWorld world(cfg);
    world.set_bodies({});
    world.step();  // must not crash
    assert(world.body_count() == 0);
    assert(world.bodies().empty());
}

void test_zero_mass_body() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.0f;  // test with epsilon=0 but non-zero distance
    cfg.dt = 0.01f;
    cfg.integrator = "rk4";
    PhysicsWorld world(cfg);

    // Body 1 is massless test particle at (0, 0), Body 2 is heavy at (100, 0)
    world.set_bodies({
        {1, 0.0f, {0.f, 0.f}, {0.f, 0.f}, 1.f},
        {2, 1000.f, {100.f, 0.f}, {0.f, 0.f}, 1.f}
    });

    world.step();
    auto& b = world.bodies();

    // Massless body should accelerate toward heavy body (x > 0)
    assert(b[0].position.x > 0.f);
    assert(b[0].velocity.x > 0.f);
    assert(!std::isnan(b[0].position.x) && !std::isinf(b[0].position.x));
    assert(!std::isnan(b[0].velocity.x) && !std::isinf(b[0].velocity.x));

    // Heavy body should experience 0 acceleration from massless body (remains at rest)
    assert(std::abs(b[1].position.x - 100.f) < 1e-5f);
    assert(std::abs(b[1].velocity.x - 0.f) < 1e-5f);
}

void test_collision_non_colliding_bodies() {
    SimConfig cfg;
    cfg.G = 0.0f;  // no gravity to isolate collision logic
    cfg.dt = 0.01f;
    cfg.integrator = "euler";
    PhysicsWorld world(cfg);
    world.set_bodies({
        {1, 10.f, {0.f, 0.f}, {0.f, 0.f}, 2.f},
        {2, 10.f, {10.f, 0.f}, {0.f, 0.f}, 2.f}
    });
    world.step();
    assert(world.body_count() == 2);
}

void test_integrator_default_fallback() {
    SimConfig cfg;
    cfg.integrator = "unknown_integrator";
    cfg.G = 1.0f;
    cfg.dt = 0.01f;
    PhysicsWorld world(cfg);
    world.set_bodies({
        {1, 10.f, {0.f, 0.f}, {1.f, 0.f}, 1.f}
    });
    world.step();
    // Default fallback to Euler should execute cleanly
    assert(world.body_count() == 1);
}

void test_set_bodies_and_accessors() {
    SimConfig cfg;
    PhysicsWorld world(cfg);
    assert(world.body_count() == 0);
    assert(world.bodies().empty());

    world.set_bodies({
        {1, 5.f, {1.f, 2.f}, {3.f, 4.f}, 0.5f},
        {2, 8.f, {5.f, 6.f}, {7.f, 8.f}, 1.2f}
    });
    assert(world.body_count() == 2);
    assert(world.bodies()[0].id == 1);
    assert(world.bodies()[1].id == 2);
}

void test_circular_orbit_stability_rk4() {
    SimConfig cfg;
    cfg.G = 1.0f;
    cfg.epsilon = 0.1f;
    cfg.dt = 0.001f;
    cfg.integrator = "rk4";
    PhysicsWorld world(cfg);
    float M = 10000.f, m = 1.f, d = 200.f;
    float v_orbit = std::sqrt(cfg.G * M / d);
    world.set_bodies({
        {1, M, {400.f, 360.f}, {0.f, 0.f}, 10.f},
        {2, m, {400.f + d, 360.f}, {0.f, v_orbit}, 2.f}
    });
    float initial_dist = d;
    // Run for many steps (~1 orbit period)
    int steps = static_cast<int>(2.0f * 3.14159f * d / v_orbit / cfg.dt);
    for (int i = 0; i < steps; ++i) {
        world.step();
    }
    // After one full orbit, distance from center should be close to initial
    auto& b = world.bodies();
    assert(b.size() == 2);
    float dx = b[1].position.x - b[0].position.x;
    float dy = b[1].position.y - b[0].position.y;
    float final_dist = std::sqrt(dx*dx + dy*dy);
    // Allow 5% deviation for RK4 with this dt
    assert(std::abs(final_dist - initial_dist) / initial_dist < 0.05f);
}

void test_zero_bodies_safety() {
    for (const auto& integrator : {"euler", "rk4"}) {
        SimConfig cfg;
        cfg.integrator = integrator;
        PhysicsWorld world(cfg);
        world.set_bodies({});
        for (int i = 0; i < 10; ++i) {
            world.step();
        }
        assert(world.body_count() == 0);
        assert(world.bodies().empty());
    }
}

int main() {
    test_two_body_attraction_euler();
    test_two_body_attraction_rk4();
    test_circular_orbit_stability_rk4();
    test_collision_merge_reduces_count();
    test_collision_chain_merge();
    test_single_body_no_crash();
    test_single_body_euler();
    test_empty_world();
    test_zero_bodies_safety();
    test_zero_mass_body();
    test_collision_non_colliding_bodies();
    test_integrator_default_fallback();
    test_set_bodies_and_accessors();

    std::cout << "All physics tests passed" << std::endl;
    return 0;
}
