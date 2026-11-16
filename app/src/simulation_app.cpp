#include "simulation_app.hpp"
#include "sim_presets.hpp"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Text.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/Window/VideoMode.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

/**
 * @file simulation_app.cpp
 * @brief Implementation of SimulationApp and application event/render loops.
 */

namespace {

/**
 * @brief Helper determining body render color based on mass threshold.
 * Massive central bodies (stars) render in yellow; smaller bodies render in white.
 */
sf::Color color_for_body(const Body& body) {
    if (body.mass > 1000.0f) {
        return sf::Color::Yellow;
    }
    return sf::Color(body.color.r, body.color.g, body.color.b);
}

} // namespace

SimulationApp::SimulationApp(SimConfig config)
    : config_(std::move(config)),
      world_(config_),
      window_(sf::VideoMode({config_.window_width, config_.window_height}), "N-Body Simulator"),
      starfield_(config_.window_width, config_.window_height),
      menu_bar_(static_cast<float>(config_.window_width), 35.0f) {
    window_.setFramerateLimit(60);
    window_.setKeyRepeatEnabled(false);

    // Probe standard cross-platform system font directories
    const std::vector<std::string> font_candidates = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "C:\\Windows\\Fonts\\arial.ttf"
    };

    std::error_code ec;
    for (const auto& path : font_candidates) {
        if (std::filesystem::exists(path, ec) && !ec && font_.openFromFile(path)) {
            font_loaded_ = true;
            break;
        }
    }

    if (!font_loaded_) {
        std::cerr << "Warning: Could not load any font. Menu bar text labels will be disabled.\n";
    }

    // Initialize MenuBar layout with font pointer (if loaded)
    menu_bar_ = MenuBar(static_cast<float>(config_.window_width), 35.0f, font_loaded_ ? &font_ : nullptr);

    // Sync initial state indicators
    menu_bar_.set_selected_scenario(config_.preset);
    menu_bar_.set_selected_integrator(config_.integrator);
    menu_bar_.set_selected_dt(config_.dt);
    menu_bar_.set_paused(paused_);

    // Wire UI callbacks to application/physics actions
    menu_bar_.set_on_scenario_changed([this](const std::string& scenario) {
        reset_preset(scenario);
    });

    menu_bar_.set_on_integrator_changed([this](const std::string& integrator) {
        world_.set_integrator(integrator);
    });

    menu_bar_.set_on_timestep_changed([this](float dt) {
        world_.set_dt(dt);
    });

    menu_bar_.set_on_pause_toggled([this]() {
        paused_ = !paused_;
        menu_bar_.set_paused(paused_);
        title_timer_ = sf::seconds(0.5f); // Force immediate window title update
    });

    // Populate bodies for starting preset and initialize title
    reset_preset(config_.preset);
    update_title_bar(sf::Time::Zero);
}

void SimulationApp::reset_preset(const std::string& preset_name) {
    config_.preset = preset_name;
    menu_bar_.set_selected_scenario(preset_name);
    if (preset_name == "solar") {
        world_.set_bodies(presets::create_solar_system(config_));
    } else if (preset_name == "random") {
        world_.set_bodies(presets::create_random_cloud(config_, 0));
    } else {
        world_.set_bodies(presets::create_stable_orbit(config_));
    }
    initial_energy_ = world_.compute_total_energy();
}

double SimulationApp::energy_percent_diff() const {
    if (std::abs(initial_energy_) < 1e-12) {
        return 0.0;
    }
    const double current_e = world_.compute_total_energy();
    return ((current_e - initial_energy_) / std::abs(initial_energy_)) * 100.0;
}

void SimulationApp::handle_events() {
    // SFML 3 std::optional event polling model
    while (const std::optional event = window_.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_.close();
        } else if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
            // Forward mouse click to MenuBar; if consumed, do not pass to simulation
            const sf::Vector2f pos(static_cast<float>(mouse->position.x), static_cast<float>(mouse->position.y));
            if (menu_bar_.handle_mouse_pressed(pos, mouse->button)) {
                // Event consumed by menu bar
            }
        } else if (const auto* move = event->getIf<sf::Event::MouseMoved>()) {
            // Forward mouse hover movement to MenuBar
            const sf::Vector2f pos(static_cast<float>(move->position.x), static_cast<float>(move->position.y));
            menu_bar_.handle_mouse_moved(pos);
        } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            // Keyboard shortcut handling
            if (key->scancode == sf::Keyboard::Scancode::Escape || key->code == sf::Keyboard::Key::Escape) {
                window_.close();
            } else if (key->scancode == sf::Keyboard::Scancode::Space || key->code == sf::Keyboard::Key::Space) {
                paused_ = !paused_;
                menu_bar_.set_paused(paused_);
                title_timer_ = sf::seconds(0.5f); // Force immediate title update
            }
        }
    }
}

void SimulationApp::update() {
    world_.step();
}

sf::Vector2f SimulationApp::world_to_screen(sf::Vector2f world_pos) const {
    // Offset Y by menu bar height (35px) so bodies render below the top bar
    return {world_pos.x, world_pos.y + menu_bar_.height()};
}

void SimulationApp::update_title_bar(sf::Time elapsed) {
    title_timer_ += elapsed;
    if (title_timer_ >= sf::seconds(0.5f)) {
        title_timer_ -= sf::seconds(0.5f);
        const std::string title =
            "N-Body Sim | FPS: " + std::to_string(fps_display_) +
            " | UPS: " + std::to_string(ups_display_) +
            " | Bodies: " + std::to_string(world_.body_count()) +
            (paused_ ? " [PAUSED]" : "");
        window_.setTitle(title);
    }
}

void SimulationApp::render() {
    window_.clear(sf::Color::Black);

    // 1. Draw static background starfield
    starfield_.render(window_);

    // 2. Draw celestial bodies in physics space, projected via world_to_screen
    for (const auto& body : world_.bodies()) {
        sf::CircleShape shape(body.radius);
        shape.setFillColor(color_for_body(body));
        shape.setOrigin({body.radius, body.radius});
        shape.setPosition(world_to_screen(body.position));
        window_.draw(shape);
    }

    // 3. Draw live energy readout below menu bar
    render_energy_overlay();

    // 4. Draw top menu bar and any open dropdown overlay on top of simulation
    menu_bar_.render(window_);

    window_.display();
}

void SimulationApp::render_energy_overlay() {
    if (!font_loaded_) {
        return;
    }
    const double current_e = world_.compute_total_energy();
    const double pct_diff = energy_percent_diff();

    std::ostringstream oss;
    oss << std::scientific << std::setprecision(4);
    oss << "Energy: " << current_e << " | Initial: " << initial_energy_ << " | Diff: ";
    oss << std::fixed << std::setprecision(2) << (pct_diff >= 0.0 ? "+" : "") << pct_diff << "%";

    sf::Text text(font_, oss.str(), 14);
    text.setFillColor(sf::Color(180, 220, 255));
    text.setPosition({15.0f, menu_bar_.height() + 8.0f});
    window_.draw(text);
}

void SimulationApp::run() {
    sf::Clock clock;
    sf::Time accumulator = sf::Time::Zero;
    sf::Time fps_timer = sf::Time::Zero;

    while (window_.isOpen()) {
        sf::Time frame_time = clock.restart();

        // Clamp frame_time to 0.1s to avoid spiral-of-death under CPU load
        if (frame_time > sf::seconds(0.1f)) {
            frame_time = sf::seconds(0.1f);
        }

        // Aggregate FPS and UPS metrics over 1-second rolling windows
        fps_timer += frame_time;
        if (fps_timer >= sf::seconds(1.0f)) {
            fps_display_ = fps_count_;
            ups_display_ = ups_count_;
            fps_count_ = 0;
            ups_count_ = 0;
            fps_timer -= sf::seconds(1.0f);
        }

        update_title_bar(frame_time);

        handle_events();
        if (!window_.isOpen()) {
            break;
        }

        // Fixed-timestep physics integration decoupled from render frame rate
        if (!paused_) {
            const float dt = (world_.dt() > 0.0f) ? world_.dt() : 0.001f;
            const sf::Time physics_dt = sf::seconds(dt);
            accumulator += frame_time;
            while (accumulator >= physics_dt) {
                update();
                accumulator -= physics_dt;
                ++ups_count_;
            }
        }

        render();
        ++fps_count_;
    }
}
