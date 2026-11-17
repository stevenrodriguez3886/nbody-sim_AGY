#include "body.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_collision_detection() {
    Body a{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 5.f};
    Body b{2, 1.f, {8.f, 0.f}, {0.f, 0.f}, 5.f};
    assert(check_collision(a, b) == true);   // distance 8 < 5+5
    assert(check_collision(b, a) == true);   // symmetry

    Body c{3, 1.f, {20.f, 0.f}, {0.f, 0.f}, 5.f};
    assert(check_collision(a, c) == false);  // distance 20 > 5+5
    assert(check_collision(c, a) == false);

    // Touching boundary: distance exactly 10 == 5 + 5
    // Spec defines collision as |r_ij| < r_i + r_j (strict inequality)
    Body d{4, 1.f, {10.f, 0.f}, {0.f, 0.f}, 5.f};
    assert(check_collision(a, d) == false);
}

void test_elastic_collision_head_on_equal_mass() {
    Body a{1, 10.0f, {0.f, 0.f}, {2.f, 0.f}, 5.f};
    Body b{2, 10.0f, {8.f, 0.f}, {-3.f, 0.f}, 5.f};

    const float initial_px = a.mass * a.velocity.x + b.mass * b.velocity.x;
    const float initial_py = a.mass * a.velocity.y + b.mass * b.velocity.y;
    const float initial_ke = 0.5f * a.mass * (a.velocity.x * a.velocity.x + a.velocity.y * a.velocity.y) +
                             0.5f * b.mass * (b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y);

    resolve_elastic_collision(a, b);

    // Equal mass head-on collision: velocities along line of centers swap
    assert(std::abs(a.velocity.x - (-3.0f)) < 1e-4f);
    assert(std::abs(a.velocity.y - 0.0f) < 1e-4f);
    assert(std::abs(b.velocity.x - 2.0f) < 1e-4f);
    assert(std::abs(b.velocity.y - 0.0f) < 1e-4f);

    // Positions are not shifted (no geometric overlap correction)
    assert(a.position.x == 0.0f && a.position.y == 0.0f);
    assert(b.position.x == 8.0f && b.position.y == 0.0f);

    // Both bodies retain their identity, mass, radius
    assert(a.id == 1 && b.id == 2);
    assert(a.mass == 10.0f && b.mass == 10.0f);
    assert(a.radius == 5.0f && b.radius == 5.0f);

    // Total momentum conserved
    const float final_px = a.mass * a.velocity.x + b.mass * b.velocity.x;
    const float final_py = a.mass * a.velocity.y + b.mass * b.velocity.y;
    assert(std::abs(final_px - initial_px) < 1e-4f);
    assert(std::abs(final_py - initial_py) < 1e-4f);

    // Total kinetic energy conserved
    const float final_ke = 0.5f * a.mass * (a.velocity.x * a.velocity.x + a.velocity.y * a.velocity.y) +
                           0.5f * b.mass * (b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y);
    assert(std::abs(final_ke - initial_ke) < 1e-4f);
}

void test_elastic_collision_unequal_mass_oblique() {
    Body a{1, 5.0f, {0.f, 0.f}, {4.f, 2.f}, 4.f};
    Body b{2, 15.0f, {6.f, 6.f}, {-1.f, -1.f}, 6.f}; // overlap: distance = sqrt(72) ≈ 8.485 < 10.0

    const float initial_px = a.mass * a.velocity.x + b.mass * b.velocity.x;
    const float initial_py = a.mass * a.velocity.y + b.mass * b.velocity.y;
    const float initial_ke = 0.5f * a.mass * (a.velocity.x * a.velocity.x + a.velocity.y * a.velocity.y) +
                             0.5f * b.mass * (b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y);

    resolve_elastic_collision(a, b);

    // Positions remain unchanged
    assert(a.position.x == 0.0f && a.position.y == 0.0f);
    assert(b.position.x == 6.0f && b.position.y == 6.0f);

    // Momentum conservation
    const float final_px = a.mass * a.velocity.x + b.mass * b.velocity.x;
    const float final_py = a.mass * a.velocity.y + b.mass * b.velocity.y;
    assert(std::abs(final_px - initial_px) < 1e-3f);
    assert(std::abs(final_py - initial_py) < 1e-3f);

    // Kinetic energy conservation
    const float final_ke = 0.5f * a.mass * (a.velocity.x * a.velocity.x + a.velocity.y * a.velocity.y) +
                           0.5f * b.mass * (b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y);
    assert(std::abs(final_ke - initial_ke) < 1e-3f);
}

void test_elastic_collision_separating_particles() {
    // Two particles overlapping, but already moving away from each other
    Body a{1, 10.0f, {0.f, 0.f}, {-2.f, 0.f}, 5.f};
    Body b{2, 10.0f, {8.f, 0.f}, {2.f, 0.f}, 5.f};

    resolve_elastic_collision(a, b);

    // Positions and velocities should remain unchanged
    assert(a.position.x == 0.f && a.position.y == 0.f);
    assert(b.position.x == 8.f && b.position.y == 0.f);
    assert(a.velocity.x == -2.f);
    assert(b.velocity.x == 2.f);
}

void test_elastic_collision_zero_mass() {
    Body a{1, 0.0f, {0.f, 0.f}, {2.f, 0.f}, 5.f};
    Body b{2, 0.0f, {8.f, 0.f}, {-2.f, 0.f}, 5.f};

    resolve_elastic_collision(a, b);

    // Should not NaN or crash, positions unchanged
    assert(a.position.x == 0.f && a.position.y == 0.f);
    assert(b.position.x == 8.f && b.position.y == 0.f);
    assert(!std::isnan(a.velocity.x) && !std::isnan(a.velocity.y));
    assert(!std::isnan(b.velocity.x) && !std::isnan(b.velocity.y));
    assert(a.velocity.x == -2.f && b.velocity.x == 2.f);
}

void test_elastic_collision_concentric() {
    // Bodies placed at the exact same point
    Body a{1, 10.0f, {5.f, 5.f}, {1.f, 0.f}, 3.f};
    Body b{2, 10.0f, {5.f, 5.f}, {-1.f, 0.f}, 3.f};

    resolve_elastic_collision(a, b);

    assert(a.position.x == 5.f && a.position.y == 5.f);
    assert(b.position.x == 5.f && b.position.y == 5.f);
    assert(!std::isnan(a.velocity.x) && !std::isnan(a.velocity.y));
    assert(!std::isnan(b.velocity.x) && !std::isnan(b.velocity.y));
}

int main() {
    test_collision_detection();
    test_elastic_collision_head_on_equal_mass();
    test_elastic_collision_unequal_mass_oblique();
    test_elastic_collision_separating_particles();
    test_elastic_collision_zero_mass();
    test_elastic_collision_concentric();
    std::cout << "All body tests passed" << std::endl;
    return 0;
}
