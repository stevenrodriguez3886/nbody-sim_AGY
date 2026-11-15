#pragma once

#include "menu_bar.hpp"
#include "physics_world.hpp"
#include "sim_config.hpp"
#include "starfield.hpp"

#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/System/Vector2.hpp>

#include <string>

class SimulationApp {
public:
    explicit SimulationApp(SimConfig config);
    void run();  // enters main loop, returns on window close

    // Accessors for testing and state inspection
    const PhysicsWorld& world() const { return world_; }
    PhysicsWorld& world_mut() { return world_; }
    bool is_paused() const { return paused_; }
    bool is_font_loaded() const { return font_loaded_; }
    const MenuBar& menu_bar() const { return menu_bar_; }
    const Starfield& starfield() const { return starfield_; }

    void reset_preset(const std::string& preset_name);
    void update_title_bar(sf::Time elapsed);
    sf::Vector2f world_to_screen(sf::Vector2f world_pos) const;

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
    sf::Time title_timer_ = sf::seconds(0.5f);
    Starfield starfield_;
    MenuBar menu_bar_;

    void handle_events();
    void update();
    void render();
};
