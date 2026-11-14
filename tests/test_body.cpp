#include "body.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_merge_mass_and_momentum() {
    Body a{1, 10.0f, {0.f, 2.f}, {1.f, -1.f}, 5.f};
    Body b{2, 20.0f, {10.f, 8.f}, {-1.f, 2.f}, 3.f};
    Body m = merge(a, b);
    assert(m.mass == 30.0f);
    // CoM position x: (10*0 + 20*10)/30 = 200/30 ≈ 6.6667
    assert(std::abs(m.position.x - 6.6667f) < 0.01f);
    // CoM position y: (10*2 + 20*8)/30 = 180/30 = 6.0
    assert(std::abs(m.position.y - 6.0f) < 0.01f);
    // Momentum-conserving velocity x: (10*1 + 20*(-1))/30 = -10/30 ≈ -0.3333
    assert(std::abs(m.velocity.x - (-0.3333f)) < 0.01f);
    // Momentum-conserving velocity y: (10*(-1) + 20*2)/30 = 30/30 = 1.0
    assert(std::abs(m.velocity.y - 1.0f) < 0.01f);
    // Area-conserving radius: sqrt(25 + 9) = sqrt(34) ≈ 5.831
    assert(std::abs(m.radius - std::sqrt(34.f)) < 0.01f);
    // ID of more massive body
    assert(m.id == 2);

    // Merge in reverse order should yield identical properties
    Body m_rev = merge(b, a);
    assert(m_rev.mass == m.mass);
    assert(std::abs(m_rev.position.x - m.position.x) < 1e-5f);
    assert(std::abs(m_rev.position.y - m.position.y) < 1e-5f);
    assert(std::abs(m_rev.velocity.x - m.velocity.x) < 1e-5f);
    assert(std::abs(m_rev.velocity.y - m.velocity.y) < 1e-5f);
    assert(std::abs(m_rev.radius - m.radius) < 1e-5f);
    assert(m_rev.id == m.id);
}

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

void test_merge_equal_mass_preserves_id() {
    Body a{1, 10.0f, {0.f, 0.f}, {0.f, 0.f}, 5.f};
    Body b{2, 10.0f, {1.f, 0.f}, {0.f, 0.f}, 5.f};
    Body m = merge(a, b);
    assert(m.id == 1);  // equal mass -> lower ID wins

    Body m2 = merge(b, a);
    assert(m2.id == 1); // order independent
}

void test_merge_zero_mass() {
    // Both zero mass
    Body a{10, 0.0f, {0.f, 0.f}, {2.f, 0.f}, 3.f};
    Body b{20, 0.0f, {4.f, 6.f}, {0.f, 4.f}, 4.f};
    Body m = merge(a, b);
    assert(m.mass == 0.0f);
    assert(m.id == 10); // equal mass -> lower ID
    assert(std::abs(m.radius - 5.0f) < 0.01f); // sqrt(9 + 16) = 5
    // Should not produce NaN/Inf in position/velocity
    assert(!std::isnan(m.position.x) && !std::isnan(m.position.y));
    assert(!std::isnan(m.velocity.x) && !std::isnan(m.velocity.y));
}

int main() {
    test_merge_mass_and_momentum();
    test_collision_detection();
    test_merge_equal_mass_preserves_id();
    test_merge_zero_mass();
    std::cout << "All body tests passed" << std::endl;
    return 0;
}
