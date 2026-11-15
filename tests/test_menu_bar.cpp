#include "menu_bar.hpp"

#include <SFML/Graphics/RenderTexture.hpp>

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

void test_initial_state() {
    MenuBar bar(1280.0f, 35.0f);
    assert(bar.height() == 35.0f);
    assert(bar.selected_scenario() == "orbit");
    assert(bar.selected_integrator() == "euler");
    assert(std::abs(bar.selected_dt() - 0.001f) < 1e-6f);
    assert(!bar.is_paused());
    assert(!bar.is_any_dropdown_open());
}

void test_scenario_dropdown_and_selection() {
    MenuBar bar(1280.0f, 35.0f);

    std::string changed_scenario;
    bar.set_on_scenario_changed([&](const std::string& scn) {
        changed_scenario = scn;
    });

    // Click on Scenario header (at x ≈ 10 to 150, y = 17.5f)
    bool consumed = bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(bar.is_any_dropdown_open());

    // Click on option "Solar System" (row 1: y ≈ 35 + 28 + 14 = 77.0f)
    consumed = bar.handle_mouse_pressed({50.0f, 77.0f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(changed_scenario == "solar");
    assert(bar.selected_scenario() == "solar");
    assert(!bar.is_any_dropdown_open());

    // Toggle dropdown open and closed by clicking header again
    consumed = bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(bar.is_any_dropdown_open());

    consumed = bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(!bar.is_any_dropdown_open());
}

void test_integrator_dropdown_and_selection() {
    MenuBar bar(1280.0f, 35.0f);

    std::string changed_integrator;
    bar.set_on_integrator_changed([&](const std::string& ig) {
        changed_integrator = ig;
    });

    // Click on Integrator header (at x ≈ 160 to 280, y = 17.5f)
    bool consumed = bar.handle_mouse_pressed({200.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(bar.is_any_dropdown_open());

    // Click on option "RK4" (row 1: y ≈ 35 + 28 + 14 = 77.0f)
    consumed = bar.handle_mouse_pressed({200.0f, 77.0f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(changed_integrator == "rk4");
    assert(bar.selected_integrator() == "rk4");
    assert(!bar.is_any_dropdown_open());
}

void test_timestep_dropdown_and_selection() {
    MenuBar bar(1280.0f, 35.0f);

    float changed_dt = 0.0f;
    bar.set_on_timestep_changed([&](float dt) {
        changed_dt = dt;
    });

    // Click on Timestep header (at x ≈ 300 to 420, y = 17.5f)
    bool consumed = bar.handle_mouse_pressed({340.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(bar.is_any_dropdown_open());

    // Click on option "0.005" (row 3: y ≈ 35 + 3 * 28 + 14 = 133.0f)
    consumed = bar.handle_mouse_pressed({340.0f, 133.0f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(std::abs(changed_dt - 0.005f) < 1e-6f);
    assert(std::abs(bar.selected_dt() - 0.005f) < 1e-6f);
    assert(!bar.is_any_dropdown_open());
}

void test_outside_click_dismiss() {
    MenuBar bar(1280.0f, 35.0f);

    // Open Scenario dropdown
    bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(bar.is_any_dropdown_open());

    // Click in simulation area outside menu bar and dropdown (e.g. x = 500, y = 300)
    bool consumed = bar.handle_mouse_pressed({500.0f, 300.0f}, sf::Mouse::Button::Left);
    assert(consumed); // Must consume click that dismisses dropdown
    assert(!bar.is_any_dropdown_open());

    // Click again in simulation area when no dropdown is open
    consumed = bar.handle_mouse_pressed({500.0f, 300.0f}, sf::Mouse::Button::Left);
    assert(!consumed); // Should NOT be consumed by MenuBar
}

void test_pause_button() {
    MenuBar bar(1280.0f, 35.0f);

    int pause_toggled_count = 0;
    bar.set_on_pause_toggled([&]() {
        ++pause_toggled_count;
    });

    // Pause button is right aligned at width - 120.0f (x ≈ 1160 to 1270, y ≈ 17.5f)
    bool consumed = bar.handle_mouse_pressed({1200.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(pause_toggled_count == 1);
    assert(bar.is_paused());

    // Click again to resume
    consumed = bar.handle_mouse_pressed({1200.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(consumed);
    assert(pause_toggled_count == 2);
    assert(!bar.is_paused());

    // Manual setter
    bar.set_paused(true);
    assert(bar.is_paused());
}

void test_hover_detection() {
    MenuBar bar(1280.0f, 35.0f);

    // Hover Scenario header
    bar.handle_mouse_moved({50.0f, 17.5f});
    assert(bar.hovered_header() == 0);

    // Hover Integrator header
    bar.handle_mouse_moved({200.0f, 17.5f});
    assert(bar.hovered_header() == 1);

    // Hover Timestep header
    bar.handle_mouse_moved({340.0f, 17.5f});
    assert(bar.hovered_header() == 2);

    // Hover Pause button
    bar.handle_mouse_moved({1200.0f, 17.5f});
    assert(bar.hovered_header() == 3);
    assert(bar.is_pause_hovered());

    // Open Scenario dropdown and test option row hover
    bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(bar.is_any_dropdown_open());

    // Row 0 hover
    bar.handle_mouse_moved({50.0f, 45.0f});
    assert(bar.hovered_dropdown_item() == 0);

    // Row 2 hover
    bar.handle_mouse_moved({50.0f, 100.0f});
    assert(bar.hovered_dropdown_item() == 2);

    // Move outside
    bar.handle_mouse_moved({500.0f, 400.0f});
    assert(bar.hovered_header() == -1);
    assert(bar.hovered_dropdown_item() == -1);
}

void test_setters_and_close_dropdowns() {
    MenuBar bar(1280.0f, 35.0f);

    bar.set_selected_scenario("random");
    assert(bar.selected_scenario() == "random");

    bar.set_selected_integrator("rk4");
    assert(bar.selected_integrator() == "rk4");

    bar.set_selected_dt(0.01f);
    assert(std::abs(bar.selected_dt() - 0.01f) < 1e-6f);

    bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
    assert(bar.is_any_dropdown_open());

    bar.close_dropdowns();
    assert(!bar.is_any_dropdown_open());
}

void test_non_left_click() {
    MenuBar bar(1280.0f, 35.0f);

    // Right click on header should be ignored
    bool consumed = bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Right);
    assert(!consumed);
    assert(!bar.is_any_dropdown_open());

    // Right click on pause button should be ignored
    consumed = bar.handle_mouse_pressed({1200.0f, 17.5f}, sf::Mouse::Button::Right);
    assert(!consumed);
    assert(!bar.is_paused());
}

void test_headless_render() {
    MenuBar bar(1280.0f, 35.0f, nullptr);

    sf::RenderTexture texture;
    // In SFML 3, create takes Vector2u
    if (texture.resize({1280, 720})) {
        texture.clear(sf::Color::Black);
        bar.render(texture);

        // Open a dropdown and render again
        bar.handle_mouse_pressed({50.0f, 17.5f}, sf::Mouse::Button::Left);
        bar.render(texture);
        texture.display();
    }
}

int main() {
    test_initial_state();
    test_scenario_dropdown_and_selection();
    test_integrator_dropdown_and_selection();
    test_timestep_dropdown_and_selection();
    test_outside_click_dismiss();
    test_pause_button();
    test_hover_detection();
    test_setters_and_close_dropdowns();
    test_non_left_click();
    test_headless_render();

    std::cout << "All MenuBar tests passed\n";
    return 0;
}
