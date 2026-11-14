#pragma once

#include "physics_world.hpp"
#include "sim_config.hpp"

#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>

class SimulationApp {
public:
    explicit SimulationApp(SimConfig config);
    void run();  // enters main loop, returns on window close

    // Accessors for testing and state inspection
    const PhysicsWorld& world() const { return world_; }
    bool is_paused() const { return paused_; }
    bool is_font_loaded() const { return font_loaded_; }

private:
    SimConfig config_;
    PhysicsWorld world_;
    sf::RenderWindow window_;
    sf::Font font_;
    bool font_loaded_ = false;
    bool paused_ = false;
    unsigned int ups_count_ = 0;  // updates-per-second counter
    unsigned int fps_count_ = 0;  // frames-per-second counter
    unsigned int ups_display_ = 0;
    unsigned int fps_display_ = 0;

    void handle_events();
    void update(float dt);
    void render();
    void draw_hud();
    sf::Vector2f world_to_screen(sf::Vector2f world_pos) const;
};
