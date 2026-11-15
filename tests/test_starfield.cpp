#include "starfield.hpp"
#include <cassert>
#include <iostream>

void test_starfield_generation() {
    Starfield stars(1280, 720, 250, 42);
    assert(stars.star_count() == 250);
    const auto& pos = stars.positions();
    assert(pos.size() == 250);

    for (const auto& p : pos) {
        assert(p.x >= 0.0f && p.x <= 1280.0f);
        assert(p.y >= 0.0f && p.y <= 720.0f);
    }

    // Deterministic with same seed
    Starfield stars2(1280, 720, 250, 42);
    for (std::size_t i = 0; i < 250; ++i) {
        assert(stars.positions()[i] == stars2.positions()[i]);
    }
}

void test_starfield_defaults_and_variations() {
    // Default count (300) and default seed (1337)
    Starfield stars_default(1920, 1080);
    assert(stars_default.star_count() == 300);
    assert(stars_default.positions().size() == 300);

    // Different seed produces different star positions
    Starfield stars_seed42(1280, 720, 250, 42);
    Starfield stars_other(1280, 720, 250, 999);
    assert(stars_other.star_count() == 250);
    bool any_difference = false;
    for (std::size_t i = 0; i < 250; ++i) {
        if (stars_other.positions()[i] != stars_seed42.positions()[i]) {
            any_difference = true;
            break;
        }
    }
    assert(any_difference);

    // Zero stars
    Starfield stars_empty(800, 600, 0);
    assert(stars_empty.star_count() == 0);
    assert(stars_empty.positions().empty());
}

int main() {
    test_starfield_generation();
    test_starfield_defaults_and_variations();
    std::cout << "All starfield tests passed\n";
    return 0;
}
