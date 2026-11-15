#include "simulation_app.hpp"
#include "sim_presets.hpp"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/Window/VideoMode.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

sf::Color color_for_body(const Body& body) {
    if (body.mass > 1000.0f) {
        return sf::Color::Yellow;
    }
    return sf::Color::White;
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

    // Candidate system font paths
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
        std::cerr << "Warning: Could not load any font. HUD rendering will be disabled.\n";
    }

    menu_bar_ = MenuBar(static_cast<float>(config_.window_width), 35.0f, font_loaded_ ? &font_ : nullptr);

    menu_bar_.set_selected_scenario(config_.preset);
    menu_bar_.set_selected_integrator(config_.integrator);
    menu_bar_.set_selected_dt(config_.dt);
    menu_bar_.set_paused(paused_);

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
    });

    reset_preset(config_.preset);
    update_title_bar(sf::Time::Zero);
}

void SimulationApp::reset_preset(const std::string& preset_name) {
    config_.preset = preset_name;
    menu_bar_.set_selected_scenario(preset_name);
    if (preset_name == "solar") {
        world_.set_bodies(presets::create_solar_system(config_));
    } else if (preset_name == "random") {
        world_.set_bodies(presets::create_random_cloud(config_));
    } else {
        world_.set_bodies(presets::create_stable_orbit(config_));
    }
}

void SimulationApp::handle_events() {
    while (const std::optional event = window_.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_.close();
        } else if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
            sf::Vector2f pos(static_cast<float>(mouse->position.x), static_cast<float>(mouse->position.y));
            if (menu_bar_.handle_mouse_pressed(pos, mouse->button)) {
                // Event consumed by menu bar
            }
        } else if (const auto* move = event->getIf<sf::Event::MouseMoved>()) {
            sf::Vector2f pos(static_cast<float>(move->position.x), static_cast<float>(move->position.y));
            menu_bar_.handle_mouse_moved(pos);
        } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            if (key->scancode == sf::Keyboard::Scancode::Escape || key->code == sf::Keyboard::Key::Escape) {
                window_.close();
            } else if (key->scancode == sf::Keyboard::Scancode::Space || key->code == sf::Keyboard::Key::Space) {
                paused_ = !paused_;
                menu_bar_.set_paused(paused_);
            }
        }
    }
}

void SimulationApp::update() {
    world_.step();
}

sf::Vector2f SimulationApp::world_to_screen(sf::Vector2f world_pos) const {
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

    starfield_.render(window_);

    for (const auto& body : world_.bodies()) {
        sf::CircleShape shape(body.radius);
        shape.setFillColor(color_for_body(body));
        shape.setOrigin({body.radius, body.radius});
        shape.setPosition(world_to_screen(body.position));
        window_.draw(shape);
    }

    menu_bar_.render(window_);

    window_.display();
}

void SimulationApp::run() {
    sf::Clock clock;
    sf::Time accumulator = sf::Time::Zero;
    sf::Time fps_timer = sf::Time::Zero;

    while (window_.isOpen()) {
        sf::Time frame_time = clock.restart();
        // Cap frame_time to avoid spiral of death
        if (frame_time > sf::seconds(0.1f)) {
            frame_time = sf::seconds(0.1f);
        }

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
