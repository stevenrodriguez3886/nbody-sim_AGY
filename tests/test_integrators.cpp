#include "euler_integrator.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

void test_euler_constant_accel() {
    std::vector<Body> bodies = {{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 1.f}};
    AccelerationFn constant_accel = [](const auto&, const auto&) {
        return std::vector<sf::Vector2f>{{1.f, 0.f}};
    };
    EulerIntegrator euler;
    euler.step(bodies, 1.0f, constant_accel);
    // After 1 step of dt=1: v = 0 + 1*1 = 1, x = 0 + 1*0 = 0 (uses old v)
    assert(std::abs(bodies[0].velocity.x - 1.0f) < 1e-5f);
    assert(std::abs(bodies[0].velocity.y - 0.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.x - 0.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.y - 0.0f) < 1e-5f);

    euler.step(bodies, 1.0f, constant_accel);
    // After 2nd step: v = 1 + 1 = 2, x = 0 + 1*1 = 1
    assert(std::abs(bodies[0].velocity.x - 2.0f) < 1e-5f);
    assert(std::abs(bodies[0].velocity.y - 0.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.x - 1.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.y - 0.0f) < 1e-5f);

    euler.step(bodies, 1.0f, constant_accel);
    // After 3rd step: v = 2 + 1 = 3, x = 1 + 1*2 = 3
    assert(std::abs(bodies[0].velocity.x - 3.0f) < 1e-5f);
    assert(std::abs(bodies[0].velocity.y - 0.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.x - 3.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.y - 0.0f) < 1e-5f);
}

void test_euler_multiple_bodies() {
    std::vector<Body> bodies = {
        {1, 10.f, {10.f, 20.f}, {1.f, -1.f}, 2.f},
        {2, 5.f, {-5.f, 0.f}, {0.f, 4.f}, 1.f}
    };
    AccelerationFn accel_fn = [](const auto&, const auto&) {
        return std::vector<sf::Vector2f>{{2.f, 3.f}, {-1.f, 0.f}};
    };
    EulerIntegrator euler;
    euler.step(bodies, 0.5f, accel_fn);

    // Body 0:
    // v = (1 + 0.5*2, -1 + 0.5*3) = (2.0, 0.5)
    // x = (10 + 0.5*1, 20 + 0.5*(-1)) = (10.5, 19.5)
    assert(std::abs(bodies[0].velocity.x - 2.0f) < 1e-5f);
    assert(std::abs(bodies[0].velocity.y - 0.5f) < 1e-5f);
    assert(std::abs(bodies[0].position.x - 10.5f) < 1e-5f);
    assert(std::abs(bodies[0].position.y - 19.5f) < 1e-5f);

    // Body 1:
    // v = (0 + 0.5*(-1), 4 + 0.5*0) = (-0.5, 4.0)
    // x = (-5 + 0.5*0, 0 + 0.5*4) = (-5.0, 2.0)
    assert(std::abs(bodies[1].velocity.x - (-0.5f)) < 1e-5f);
    assert(std::abs(bodies[1].velocity.y - 4.0f) < 1e-5f);
    assert(std::abs(bodies[1].position.x - (-5.0f)) < 1e-5f);
    assert(std::abs(bodies[1].position.y - 2.0f) < 1e-5f);
}

void test_euler_zero_dt() {
    std::vector<Body> bodies = {
        {1, 10.f, {5.f, 15.f}, {2.f, -3.f}, 2.f}
    };
    AccelerationFn accel_fn = [](const auto&, const auto&) {
        return std::vector<sf::Vector2f>{{10.f, 20.f}};
    };
    EulerIntegrator euler;
    euler.step(bodies, 0.0f, accel_fn);
    assert(bodies[0].position.x == 5.f && bodies[0].position.y == 15.f);
    assert(bodies[0].velocity.x == 2.f && bodies[0].velocity.y == -3.f);
}

void test_euler_empty_bodies() {
    std::vector<Body> bodies;
    bool called = false;
    AccelerationFn accel_fn = [&called](const auto& pos, const auto& mass) {
        called = true;
        assert(pos.empty());
        assert(mass.empty());
        return std::vector<sf::Vector2f>{};
    };
    EulerIntegrator euler;
    euler.step(bodies, 1.0f, accel_fn);
    assert(called);
    assert(bodies.empty());
}

void test_accel_fn_receives_correct_data() {
    std::vector<Body> bodies = {
        {1, 12.5f, {3.f, 4.f}, {0.f, 0.f}, 1.f},
        {2, 25.0f, {-1.f, -2.f}, {0.f, 0.f}, 1.f}
    };
    bool verified = false;
    AccelerationFn accel_fn = [&verified](const auto& pos, const auto& mass) {
        assert(pos.size() == 2);
        assert(mass.size() == 2);
        assert(pos[0].x == 3.f && pos[0].y == 4.f);
        assert(pos[1].x == -1.f && pos[1].y == -2.f);
        assert(mass[0] == 12.5f);
        assert(mass[1] == 25.0f);
        verified = true;
        return std::vector<sf::Vector2f>{{0.f, 0.f}, {0.f, 0.f}};
    };
    EulerIntegrator euler;
    euler.step(bodies, 0.1f, accel_fn);
    assert(verified);
}

void test_polymorphic_usage() {
    std::unique_ptr<IIntegrator> integrator = std::make_unique<EulerIntegrator>();
    std::vector<Body> bodies = {{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 1.f}};
    AccelerationFn accel_fn = [](const auto&, const auto&) {
        return std::vector<sf::Vector2f>{{5.f, 0.f}};
    };
    integrator->step(bodies, 1.0f, accel_fn);
    assert(std::abs(bodies[0].velocity.x - 5.0f) < 1e-5f);
    assert(std::abs(bodies[0].position.x - 0.0f) < 1e-5f);
}

int main() {
    test_euler_constant_accel();
    test_euler_multiple_bodies();
    test_euler_zero_dt();
    test_euler_empty_bodies();
    test_accel_fn_receives_correct_data();
    test_polymorphic_usage();
    std::cout << "All integrator tests passed" << std::endl;
    return 0;
}
