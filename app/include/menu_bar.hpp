#pragma once

#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Mouse.hpp>

#include <functional>
#include <string>
#include <vector>

class MenuBar {
public:
    MenuBar(float width, float height, const sf::Font* font = nullptr);

    using SelectionCallback = std::function<void(const std::string&)>;
    using TimestepCallback = std::function<void(float)>;
    using ToggleCallback = std::function<void()>;

    void set_on_scenario_changed(SelectionCallback cb);
    void set_on_integrator_changed(SelectionCallback cb);
    void set_on_timestep_changed(TimestepCallback cb);
    void set_on_pause_toggled(ToggleCallback cb);

    void set_selected_scenario(const std::string& scenario);
    void set_selected_integrator(const std::string& integrator);
    void set_selected_dt(float dt);
    void set_paused(bool paused);

    bool handle_mouse_pressed(sf::Vector2f mouse_pos, sf::Mouse::Button button);
    void handle_mouse_moved(sf::Vector2f mouse_pos);

    void render(sf::RenderTarget& target) const;

    float height() const;
    float width() const;
    bool is_any_dropdown_open() const;
    void close_dropdowns();
    const std::string& selected_scenario() const;
    const std::string& selected_integrator() const;
    float selected_dt() const;
    bool is_paused() const;

    // Inspection helpers for tests
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
