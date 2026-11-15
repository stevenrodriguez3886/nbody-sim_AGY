#pragma once

#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Mouse.hpp>

#include <functional>
#include <string>
#include <vector>

/**
 * @file menu_bar.hpp
 * @brief Interactive top menu bar and dropdown widget system for runtime simulation controls.
 */

/**
 * @class MenuBar
 * @brief Renders a top control bar (~35px) and manages interactive dropdown menus and buttons.
 *
 * Provides clickable headers for:
 * - Scenario: Orbit, Solar System, Random Cloud
 * - Integrator: Euler, RK4
 * - Timestep: 0.0001, 0.0005, 0.001, 0.005, 0.01
 * - Pause / Resume toggle button
 *
 * Handles mouse press and motion events, bounding box hit-testing, active indicators,
 * hover states, and outside-click dismissal.
 */
class MenuBar {
public:
    /**
     * @brief Constructs a MenuBar with given dimensions and optional font reference.
     * @param width Bar width (typically matching window width).
     * @param height Bar height (default 35.0f pixels).
     * @param font Pointer to loaded sf::Font for rendering labels (nullptr disables text drawing safely).
     */
    MenuBar(float width, float height, const sf::Font* font = nullptr);

    using SelectionCallback = std::function<void(const std::string&)>;  ///< Callback for string-based selections.
    using TimestepCallback = std::function<void(float)>;                ///< Callback for numeric timestep changes.
    using ToggleCallback = std::function<void()>;                       ///< Callback for button toggle triggers.

    /** @brief Sets callback invoked when user selects a simulation scenario. */
    void set_on_scenario_changed(SelectionCallback cb);

    /** @brief Sets callback invoked when user selects an integrator algorithm. */
    void set_on_integrator_changed(SelectionCallback cb);

    /** @brief Sets callback invoked when user selects a new timestep dt. */
    void set_on_timestep_changed(TimestepCallback cb);

    /** @brief Sets callback invoked when user clicks the Pause/Resume button. */
    void set_on_pause_toggled(ToggleCallback cb);

    /** @brief Synchronizes active scenario indicator without invoking callback. */
    void set_selected_scenario(const std::string& scenario);

    /** @brief Synchronizes active integrator indicator without invoking callback. */
    void set_selected_integrator(const std::string& integrator);

    /** @brief Synchronizes active timestep indicator without invoking callback. */
    void set_selected_dt(float dt);

    /** @brief Synchronizes pause button state and visual label. */
    void set_paused(bool paused);

    /**
     * @brief Processes mouse button press events.
     *
     * Evaluates clicks against the menu bar, open dropdowns, or outside space.
     * Clicks outside an open dropdown dismiss the dropdown and consume the click (returning true).
     *
     * @param mouse_pos Mouse pointer coordinate in screen space.
     * @param button Mouse button pressed (e.g., sf::Mouse::Button::Left).
     * @return true if the event was consumed by the MenuBar, false otherwise.
     */
    bool handle_mouse_pressed(sf::Vector2f mouse_pos, sf::Mouse::Button button);

    /**
     * @brief Processes mouse move events to update hover highlights.
     * @param mouse_pos Mouse pointer coordinate in screen space.
     */
    void handle_mouse_moved(sf::Vector2f mouse_pos);

    /**
     * @brief Renders the menu bar, active dropdown overlay, and labels to the target.
     * @param target Destination SFML render target.
     */
    void render(sf::RenderTarget& target) const;

    float height() const;                           ///< Returns menu bar height in pixels.
    float width() const;                            ///< Returns menu bar width in pixels.
    bool is_any_dropdown_open() const;              ///< Returns true if any dropdown menu is active.
    void close_dropdowns();                         ///< Dismisses any currently open dropdown.
    const std::string& selected_scenario() const;   ///< Returns currently selected scenario identifier.
    const std::string& selected_integrator() const; ///< Returns currently selected integrator identifier.
    float selected_dt() const;                      ///< Returns currently selected timestep value.
    bool is_paused() const;                         ///< Returns current pause state.

    // Inspection helpers for testing
    int hovered_header() const;
    int hovered_dropdown_item() const;
    bool is_pause_hovered() const;

private:
    enum class DropdownType {
        None,
        Scenario,
        Integrator,
        Timestep
    };

    struct ScenarioOption {
        std::string label;
        std::string value;
    };

    struct IntegratorOption {
        std::string label;
        std::string value;
    };

    struct TimestepOption {
        std::string label;
        float value;
    };

    sf::FloatRect get_scenario_header_rect() const;
    sf::FloatRect get_integrator_header_rect() const;
    sf::FloatRect get_timestep_header_rect() const;
    sf::FloatRect get_pause_button_rect() const;
    sf::FloatRect get_dropdown_rect(DropdownType type) const;
    int get_dropdown_item_count(DropdownType type) const;

    float width_{1280.0f};
    float height_{35.0f};
    const sf::Font* font_{nullptr};

    std::string selected_scenario_{"orbit"};
    std::string selected_integrator_{"euler"};
    float selected_dt_{0.001f};
    bool paused_{false};

    DropdownType open_dropdown_{DropdownType::None};
    int hovered_header_{-1}; // 0: Scenario, 1: Integrator, 2: Timestep, 3: Pause
    int hovered_dropdown_item_{-1};

    SelectionCallback on_scenario_changed_;
    SelectionCallback on_integrator_changed_;
    TimestepCallback on_timestep_changed_;
    ToggleCallback on_pause_toggled_;

    static const std::vector<ScenarioOption> kScenarioOptions;
    static const std::vector<IntegratorOption> kIntegratorOptions;
    static const std::vector<TimestepOption> kTimestepOptions;

    static constexpr float kRowHeight{28.0f};
};
