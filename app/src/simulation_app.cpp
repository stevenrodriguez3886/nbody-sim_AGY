#include "simulation_app.hpp"
#include "sim_presets.hpp"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Text.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
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
      window_(sf::VideoMode({config_.window_width, config_.window_height}), "N-Body Simulator") {
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

    // Set bodies matching preset
    if (config_.preset == "solar") {
        world_.set_bodies(presets::create_solar_system(config_));
    } else if (config_.preset == "random") {
        world_.set_bodies(presets::create_random_cloud(config_));
    } else {
        world_.set_bodies(presets::create_stable_orbit(config_));
    }
}

void SimulationApp::handle_events() {
    while (const std::optional event = window_.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_.close();
        } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            if (key->scancode == sf::Keyboard::Scancode::Escape || key->code == sf::Keyboard::Key::Escape) {
                window_.close();
            } else if (key->scancode == sf::Keyboard::Scancode::Space || key->code == sf::Keyboard::Key::Space) {
                paused_ = !paused_;
            }
        }
    }
}

void SimulationApp::update(float dt) {
    (void)dt;
    world_.step();
}

sf::Vector2f SimulationApp::world_to_screen(sf::Vector2f world_pos) const {
    return world_pos;
}

void SimulationApp::draw_hud() {
    if (!font_loaded_) {
        return;
    }

    const std::string hud_content =
        "FPS: " + std::to_string(fps_display_) + "\n" +
        "UPS: " + std::to_string(ups_display_) + "\n" +
        "Bodies: " + std::to_string(world_.body_count()) + "\n" +
        "Status: " + (paused_ ? "PAUSED (Space to resume)" : "RUNNING (Space to pause)");

    sf::Text hud_text(font_, hud_content, 18);
    hud_text.setFillColor(sf::Color::White);
    hud_text.setPosition({10.0f, 10.0f});
    window_.draw(hud_text);
}

void SimulationApp::render() {
    window_.clear(sf::Color::Black);

    for (const auto& body : world_.bodies()) {
        sf::CircleShape shape(body.radius);
        shape.setFillColor(color_for_body(body));
        shape.setOrigin({body.radius, body.radius});
        shape.setPosition(world_to_screen(body.position));
        window_.draw(shape);
    }

    draw_hud();

    window_.display();
}

void SimulationApp::run() {
    sf::Clock clock;
    sf::Time accumulator = sf::Time::Zero;
    const float dt = (config_.dt > 0.0f) ? config_.dt : 0.001f;
    const sf::Time physics_dt = sf::seconds(dt);
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

        handle_events();
        if (!window_.isOpen()) {
            break;
        }

        if (!paused_) {
            accumulator += frame_time;
            while (accumulator >= physics_dt) {
                update(dt);
                accumulator -= physics_dt;
                ++ups_count_;
            }
        }

        render();
        ++fps_count_;
    }
}
