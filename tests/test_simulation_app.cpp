#include "simulation_app.hpp"

#include <cassert>
#include <iostream>

bool clouds_differ(const std::vector<Body>& first, const std::vector<Body>& second) {
    if (first.size() != second.size()) {
        return true;
    }

    for (size_t i = 0; i < first.size(); ++i) {
        if (first[i].position != second[i].position ||
            first[i].velocity != second[i].velocity ||
            first[i].mass != second[i].mass ||
            first[i].color.r != second[i].color.r ||
            first[i].color.g != second[i].color.g ||
            first[i].color.b != second[i].color.b) {
            return true;
        }
    }

    return false;
}

void test_app_initialization_and_reset() {
    SimConfig cfg;
    cfg.window_width = 800;
    cfg.window_height = 600;
    cfg.preset = "orbit";
    cfg.integrator = "euler";
    cfg.dt = 0.001f;

    SimulationApp app(cfg);
    assert(app.world().body_count() == 2);
    assert(!app.is_paused());

    // Test scenario resetting
    app.reset_preset("solar");
    assert(app.world().body_count() == 6);

    app.reset_preset("random");
    assert(app.world().body_count() >= 50);
    const auto first_random_cloud = app.world().bodies();

    app.reset_preset("random");
    assert(clouds_differ(first_random_cloud, app.world().bodies()));

    // Test integrator swapping on app world
    app.world_mut().set_integrator("rk4");
    assert(app.world().integrator_name() == "rk4");

    // Test timestep updating
    app.world_mut().set_dt(0.005f);
    assert(app.world().dt() == 0.005f);

    // Test window title generation / update without crash
    app.update_title_bar(sf::seconds(1.0f));

    // Test world-to-screen coordinate mapping
    const sf::Vector2f screen = app.world_to_screen({100.0f, 200.0f});
    assert(screen.x == 100.0f);
    assert(screen.y == 200.0f + app.menu_bar().height());

    // Test headless world step
    app.world_mut().step();
    assert(app.world().body_count() >= 50);
}

void test_energy_tracking() {
    SimConfig cfg;
    cfg.window_width = 800;
    cfg.window_height = 600;
    cfg.preset = "orbit";
    cfg.integrator = "euler";
    cfg.dt = 0.001f;

    SimulationApp app(cfg);
    assert(app.initial_energy() != 0.0);
    assert(std::abs(app.initial_energy() - app.world().compute_total_energy()) < 1e-5);
    assert(std::abs(app.energy_percent_diff()) < 1e-5);

    // After reset, initial energy should update to new scenario's starting energy
    app.reset_preset("solar");
    assert(std::abs(app.initial_energy() - app.world().compute_total_energy()) < 1e-5);
}

int main() {
    test_app_initialization_and_reset();
    test_energy_tracking();
    std::cout << "All simulation app integration tests passed\n";
    return 0;
}
