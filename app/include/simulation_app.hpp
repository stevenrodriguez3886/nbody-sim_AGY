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

/**
 * @file simulation_app.hpp
 * @brief Top-level SFML application orchestrating physics, rendering, UI, and event loops.
 */

/**
 * @class SimulationApp
 * @brief Main application class managing the SFML window, decoupled physics loop, top menu bar, and starfield.
 */
class SimulationApp {
public:
    /**
     * @brief Constructs the application with configuration, window, fonts, and UI widgets.
     * @param config Initial simulation settings.
     */
    explicit SimulationApp(SimConfig config);

    /**
     * @brief Enters the main execution loop (event polling, fixed-dt physics accumulator, rendering).
     * Returns when the window is closed.
     */
    void run();

    // Accessors for testing and state inspection
    const PhysicsWorld& world() const { return world_; }
    PhysicsWorld& world_mut() { return world_; }
    bool is_paused() const { return paused_; }
    bool is_font_loaded() const { return font_loaded_; }
    const MenuBar& menu_bar() const { return menu_bar_; }
    const Starfield& starfield() const { return starfield_; }
    double initial_energy() const { return initial_energy_; }
    double current_energy() const { return world_.compute_total_energy(); }
    double energy_percent_diff() const;

    /**
     * @brief Resets the simulation bodies and configuration to the specified preset.
     * @param preset_name Scenario name ("orbit", "solar", or "random").
     */
    void reset_preset(const std::string& preset_name);

    /**
     * @brief Updates the window title with FPS, UPS, body count, and pause status every ~0.5s.
     * @param elapsed Delta time elapsed since last frame.
     */
    void update_title_bar(sf::Time elapsed);

    /**
     * @brief Transforms physics coordinates to screen pixel coordinates with vertical menu offset.
     * @param world_pos Physics space position.
     * @return sf::Vector2f Projected screen pixel position.
     */
    sf::Vector2f world_to_screen(sf::Vector2f world_pos) const;

private:
    SimConfig config_;                          ///< Active simulation configuration.
    PhysicsWorld world_;                        ///< Underlying gravitation physics engine.
    sf::RenderWindow window_;                   ///< SFML main rendering window.
    sf::Font font_;                             ///< Loaded UI font asset.
    bool font_loaded_ = false;                  ///< Flag indicating if a system font was loaded successfully.
    bool paused_ = false;                       ///< Pause state toggle.
    unsigned int ups_count_ = 0;                ///< Per-second updates counter.
    unsigned int fps_count_ = 0;                ///< Per-second frame render counter.
    unsigned int ups_display_ = 0;              ///< Cached UPS metric displayed in window title.
    unsigned int fps_display_ = 0;              ///< Cached FPS metric displayed in window title.
    sf::Time title_timer_ = sf::seconds(0.5f);  ///< Timer tracking title update cadence.
    double initial_energy_ = 0.0;               ///< Total system energy at scenario start or reset.
    Starfield starfield_;                       ///< Background starry sky renderer.
    MenuBar menu_bar_;                          ///< Top interactive control bar and dropdowns.

    /** @brief Polls and dispatches SFML window, mouse, and keyboard events. */
    void handle_events();

    /** @brief Steps the physics world by one timestep dt. */
    void update();

    /** @brief Clears the window and draws the starfield, bodies, and menu bar in back-to-front order. */
    void render();

    /** @brief Renders the live system energy and percentage diff overlay below the menu bar. */
    void render_energy_overlay();
};
