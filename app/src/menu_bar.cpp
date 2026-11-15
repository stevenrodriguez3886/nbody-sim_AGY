#include "menu_bar.hpp"

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/Text.hpp>

#include <cmath>
#include <utility>

const std::vector<MenuBar::ScenarioOption> MenuBar::kScenarioOptions = {
    {"Orbit", "orbit"},
    {"Solar System", "solar"},
    {"Random Cloud", "random"}
};

const std::vector<MenuBar::IntegratorOption> MenuBar::kIntegratorOptions = {
    {"Euler", "euler"},
    {"RK4", "rk4"}
};

const std::vector<MenuBar::TimestepOption> MenuBar::kTimestepOptions = {
    {"0.0001", 0.0001f},
    {"0.0005", 0.0005f},
    {"0.001", 0.001f},
    {"0.005", 0.005f},
    {"0.01", 0.01f}
};

MenuBar::MenuBar(float width, float height, const sf::Font* font)
    : width_(width),
      height_(height),
      font_(font) {
}

void MenuBar::set_on_scenario_changed(SelectionCallback cb) {
    on_scenario_changed_ = std::move(cb);
}

void MenuBar::set_on_integrator_changed(SelectionCallback cb) {
    on_integrator_changed_ = std::move(cb);
}

void MenuBar::set_on_timestep_changed(TimestepCallback cb) {
    on_timestep_changed_ = std::move(cb);
}

void MenuBar::set_on_pause_toggled(ToggleCallback cb) {
    on_pause_toggled_ = std::move(cb);
}

void MenuBar::set_selected_scenario(const std::string& scenario) {
    selected_scenario_ = scenario;
}

void MenuBar::set_selected_integrator(const std::string& integrator) {
    selected_integrator_ = integrator;
}

void MenuBar::set_selected_dt(float dt) {
    selected_dt_ = dt;
}

void MenuBar::set_paused(bool paused) {
    paused_ = paused;
}

float MenuBar::height() const {
    return height_;
}

float MenuBar::width() const {
    return width_;
}

bool MenuBar::is_any_dropdown_open() const {
    return open_dropdown_ != DropdownType::None;
}

void MenuBar::close_dropdowns() {
    open_dropdown_ = DropdownType::None;
    hovered_dropdown_item_ = -1;
}

const std::string& MenuBar::selected_scenario() const {
    return selected_scenario_;
}

const std::string& MenuBar::selected_integrator() const {
    return selected_integrator_;
}

float MenuBar::selected_dt() const {
    return selected_dt_;
}

bool MenuBar::is_paused() const {
    return paused_;
}

int MenuBar::hovered_header() const {
    return hovered_header_;
}

int MenuBar::hovered_dropdown_item() const {
    return hovered_dropdown_item_;
}

bool MenuBar::is_pause_hovered() const {
    return hovered_header_ == 3;
}

sf::FloatRect MenuBar::get_scenario_header_rect() const {
    return sf::FloatRect({10.0f, 0.0f}, {140.0f, height_});
}

sf::FloatRect MenuBar::get_integrator_header_rect() const {
    return sf::FloatRect({160.0f, 0.0f}, {120.0f, height_});
}

sf::FloatRect MenuBar::get_timestep_header_rect() const {
    return sf::FloatRect({300.0f, 0.0f}, {120.0f, height_});
}

sf::FloatRect MenuBar::get_pause_button_rect() const {
    return sf::FloatRect({width_ - 120.0f, 4.0f}, {110.0f, height_ - 8.0f});
}

sf::FloatRect MenuBar::get_dropdown_rect(DropdownType type) const {
    switch (type) {
        case DropdownType::Scenario:
            return sf::FloatRect(
                {10.0f, height_},
                {140.0f, static_cast<float>(kScenarioOptions.size()) * kRowHeight}
            );
        case DropdownType::Integrator:
            return sf::FloatRect(
                {160.0f, height_},
                {120.0f, static_cast<float>(kIntegratorOptions.size()) * kRowHeight}
            );
        case DropdownType::Timestep:
            return sf::FloatRect(
                {300.0f, height_},
                {120.0f, static_cast<float>(kTimestepOptions.size()) * kRowHeight}
            );
        case DropdownType::None:
        default:
            return sf::FloatRect({0.0f, 0.0f}, {0.0f, 0.0f});
    }
}

int MenuBar::get_dropdown_item_count(DropdownType type) const {
    switch (type) {
        case DropdownType::Scenario:
            return static_cast<int>(kScenarioOptions.size());
        case DropdownType::Integrator:
            return static_cast<int>(kIntegratorOptions.size());
        case DropdownType::Timestep:
            return static_cast<int>(kTimestepOptions.size());
        case DropdownType::None:
        default:
            return 0;
    }
}

bool MenuBar::handle_mouse_pressed(sf::Vector2f mouse_pos, sf::Mouse::Button button) {
    if (button != sf::Mouse::Button::Left) {
        return false;
    }

    // 1. If a dropdown is open, check if the click hit inside that dropdown
    if (open_dropdown_ != DropdownType::None) {
        sf::FloatRect drop_rect = get_dropdown_rect(open_dropdown_);
        if (drop_rect.contains(mouse_pos)) {
            int idx = static_cast<int>((mouse_pos.y - height_) / kRowHeight);
            if (open_dropdown_ == DropdownType::Scenario) {
                if (idx >= 0 && idx < static_cast<int>(kScenarioOptions.size())) {
                    selected_scenario_ = kScenarioOptions[idx].value;
                    if (on_scenario_changed_) {
                        on_scenario_changed_(selected_scenario_);
                    }
                }
            } else if (open_dropdown_ == DropdownType::Integrator) {
                if (idx >= 0 && idx < static_cast<int>(kIntegratorOptions.size())) {
                    selected_integrator_ = kIntegratorOptions[idx].value;
                    if (on_integrator_changed_) {
                        on_integrator_changed_(selected_integrator_);
                    }
                }
            } else if (open_dropdown_ == DropdownType::Timestep) {
                if (idx >= 0 && idx < static_cast<int>(kTimestepOptions.size())) {
                    selected_dt_ = kTimestepOptions[idx].value;
                    if (on_timestep_changed_) {
                        on_timestep_changed_(selected_dt_);
                    }
                }
            }
            close_dropdowns();
            return true;
        }
    }

    // 2. Check if click is on the menu bar
    sf::FloatRect bar_rect({0.0f, 0.0f}, {width_, height_});
    if (bar_rect.contains(mouse_pos)) {
        // Pause button
        sf::FloatRect pause_rect = get_pause_button_rect();
        if (pause_rect.contains(mouse_pos)) {
            close_dropdowns();
            paused_ = !paused_;
            if (on_pause_toggled_) {
                on_pause_toggled_();
            }
            return true;
        }

        // Scenario header
        sf::FloatRect scn_rect = get_scenario_header_rect();
        if (scn_rect.contains(mouse_pos)) {
            open_dropdown_ = (open_dropdown_ == DropdownType::Scenario)
                                 ? DropdownType::None
                                 : DropdownType::Scenario;
            return true;
        }

        // Integrator header
        sf::FloatRect ig_rect = get_integrator_header_rect();
        if (ig_rect.contains(mouse_pos)) {
            open_dropdown_ = (open_dropdown_ == DropdownType::Integrator)
                                 ? DropdownType::None
                                 : DropdownType::Integrator;
            return true;
        }

        // Timestep header
        sf::FloatRect dt_rect = get_timestep_header_rect();
        if (dt_rect.contains(mouse_pos)) {
            open_dropdown_ = (open_dropdown_ == DropdownType::Timestep)
                                 ? DropdownType::None
                                 : DropdownType::Timestep;
            return true;
        }

        // Clicked on empty space within the menu bar
        close_dropdowns();
        return true;
    }

    // 3. Click is outside the menu bar and outside the open dropdown
    if (open_dropdown_ != DropdownType::None) {
        close_dropdowns();
        return true; // outside click dismisses dropdown and consumes event
    }

    return false;
}

void MenuBar::handle_mouse_moved(sf::Vector2f mouse_pos) {
    hovered_header_ = -1;
    hovered_dropdown_item_ = -1;

    sf::FloatRect bar_rect({0.0f, 0.0f}, {width_, height_});
    if (bar_rect.contains(mouse_pos)) {
        if (get_scenario_header_rect().contains(mouse_pos)) {
            hovered_header_ = 0;
        } else if (get_integrator_header_rect().contains(mouse_pos)) {
            hovered_header_ = 1;
        } else if (get_timestep_header_rect().contains(mouse_pos)) {
            hovered_header_ = 2;
        } else if (get_pause_button_rect().contains(mouse_pos)) {
            hovered_header_ = 3;
        }
    }

    if (open_dropdown_ != DropdownType::None) {
        sf::FloatRect drop_rect = get_dropdown_rect(open_dropdown_);
        if (drop_rect.contains(mouse_pos)) {
            int idx = static_cast<int>((mouse_pos.y - height_) / kRowHeight);
            int max_items = get_dropdown_item_count(open_dropdown_);
            if (idx >= 0 && idx < max_items) {
                hovered_dropdown_item_ = idx;
            }
        }
    }
}

void MenuBar::render(sf::RenderTarget& target) const {
    // 1. Menu bar background
    sf::RectangleShape bar({width_, height_});
    bar.setFillColor(sf::Color(30, 30, 30, 235));
    bar.setPosition({0.0f, 0.0f});
    target.draw(bar);

    sf::RectangleShape divider({width_, 1.0f});
    divider.setFillColor(sf::Color(65, 65, 65, 255));
    divider.setPosition({0.0f, height_ - 1.0f});
    target.draw(divider);

    // Helper to render a header item
    auto render_header = [&](const sf::FloatRect& rect,
                             const std::string& label,
                             int header_idx,
                             bool is_active) {
        bool is_hovered = (hovered_header_ == header_idx);
        if (is_hovered || is_active) {
            sf::RectangleShape bg(rect.size);
            bg.setPosition(rect.position);
            bg.setFillColor(is_active ? sf::Color(60, 60, 60, 240)
                                      : sf::Color(45, 45, 45, 230));
            target.draw(bg);
        }

        if (font_) {
            sf::Text text(*font_, label, 14);
            text.setFillColor(is_active ? sf::Color(100, 200, 255)
                                        : (is_hovered ? sf::Color::White : sf::Color(210, 210, 210)));
            sf::FloatRect bounds = text.getLocalBounds();
            float tx = rect.position.x + 12.0f;
            float ty = rect.position.y + (rect.size.y - bounds.size.y) / 2.0f - bounds.position.y;
            text.setPosition({tx, ty});
            target.draw(text);
        }
    };

    render_header(get_scenario_header_rect(), "Scenario", 0, open_dropdown_ == DropdownType::Scenario);
    render_header(get_integrator_header_rect(), "Integrator", 1, open_dropdown_ == DropdownType::Integrator);
    render_header(get_timestep_header_rect(), "Timestep", 2, open_dropdown_ == DropdownType::Timestep);

    // 2. Pause/Resume button
    sf::FloatRect pause_rect = get_pause_button_rect();
    sf::RectangleShape btn_bg(pause_rect.size);
    btn_bg.setPosition(pause_rect.position);
    bool pause_hovered = (hovered_header_ == 3);
    if (paused_) {
        btn_bg.setFillColor(pause_hovered ? sf::Color(190, 85, 45, 245)
                                          : sf::Color(160, 65, 30, 230));
    } else {
        btn_bg.setFillColor(pause_hovered ? sf::Color(65, 65, 65, 245)
                                          : sf::Color(45, 45, 45, 220));
    }
    btn_bg.setOutlineThickness(1.0f);
    btn_bg.setOutlineColor(sf::Color(85, 85, 85, 255));
    target.draw(btn_bg);

    if (font_) {
        std::string btn_label = paused_ ? "Resume" : "Pause";
        sf::Text btn_text(*font_, btn_label, 14);
        btn_text.setFillColor(sf::Color::White);
        sf::FloatRect bounds = btn_text.getLocalBounds();
        float bx = pause_rect.position.x + (pause_rect.size.x - bounds.size.x) / 2.0f - bounds.position.x;
        float by = pause_rect.position.y + (pause_rect.size.y - bounds.size.y) / 2.0f - bounds.position.y;
        btn_text.setPosition({bx, by});
        target.draw(btn_text);
    }

    // 3. Dropdown (if open)
    if (open_dropdown_ != DropdownType::None) {
        sf::FloatRect drop_rect = get_dropdown_rect(open_dropdown_);
        sf::RectangleShape drop_bg(drop_rect.size);
        drop_bg.setPosition(drop_rect.position);
        drop_bg.setFillColor(sf::Color(32, 32, 32, 250));
        drop_bg.setOutlineThickness(1.0f);
        drop_bg.setOutlineColor(sf::Color(75, 75, 75, 255));
        target.draw(drop_bg);

        int count = get_dropdown_item_count(open_dropdown_);
        for (int i = 0; i < count; ++i) {
            sf::Vector2f row_pos = {drop_rect.position.x, drop_rect.position.y + static_cast<float>(i) * kRowHeight};
            sf::Vector2f row_size = {drop_rect.size.x, kRowHeight};

            bool row_hovered = (hovered_dropdown_item_ == i);
            bool row_selected = false;
            std::string label;

            if (open_dropdown_ == DropdownType::Scenario) {
                label = kScenarioOptions[i].label;
                row_selected = (selected_scenario_ == kScenarioOptions[i].value);
            } else if (open_dropdown_ == DropdownType::Integrator) {
                label = kIntegratorOptions[i].label;
                row_selected = (selected_integrator_ == kIntegratorOptions[i].value);
            } else if (open_dropdown_ == DropdownType::Timestep) {
                label = kTimestepOptions[i].label;
                row_selected = (std::abs(selected_dt_ - kTimestepOptions[i].value) < 1e-6f);
            }

            if (row_hovered) {
                sf::RectangleShape row_bg(row_size);
                row_bg.setPosition(row_pos);
                row_bg.setFillColor(sf::Color(55, 55, 55, 240));
                target.draw(row_bg);
            }

            if (row_selected) {
                sf::RectangleShape marker({3.0f, kRowHeight - 6.0f});
                marker.setPosition({row_pos.x + 2.0f, row_pos.y + 3.0f});
                marker.setFillColor(sf::Color(80, 160, 255));
                target.draw(marker);
            }

            if (font_) {
                sf::Text row_text(*font_, label, 13);
                row_text.setFillColor(row_selected ? sf::Color(100, 200, 255)
                                                   : (row_hovered ? sf::Color::White : sf::Color(210, 210, 210)));
                sf::FloatRect bounds = row_text.getLocalBounds();
                float rx = row_pos.x + 12.0f;
                float ry = row_pos.y + (kRowHeight - bounds.size.y) / 2.0f - bounds.position.y;
                row_text.setPosition({rx, ry});
                target.draw(row_text);
            }
        }
    }
}
