# Simulation Menu & Visual Polish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an interactive top menu bar with dropdowns for runtime scenario/integrator/timestep selection, a static background starfield, realistic solar system spacing, and title bar HUD tracking.

**Architecture:** Extend `PhysicsWorld` with runtime mutators (`set_integrator`, `set_dt`). Implement reusable SFML UI components in `app/` (`Starfield` and `MenuBar` with dropdowns) with event handling for mouse clicks and hover. Update `SimulationApp` to coordinate the menu bar, starfield, viewport offset, and title-bar HUD updates.

**Tech Stack:** C++17, SFML 3.0 (Graphics, Window, System), CMake, CTest.

**Spec:** `docs/superpowers/specs/2026-11-14-simulation-menu-design.md`

## Global Constraints

- Language & Standards: C++17, strictly zero warnings under `-Wall -Wextra -Wpedantic`.
- Dependencies: Only SFML 3.0 and standard library; no external GUI libraries (no ImGui).
- Architectural Separation: `physics/` library depends only on `SFML::System`; all UI components live in `app/`.
- UI Layout: Top menu bar (~35px height) with dropdowns for "Scenario", "Integrator", "Timestep", and a "Pause/Resume" button on the right.
- Coordinate Mapping: Screen positions offset vertically by `menu_bar_height` so simulation renders beneath the menu bar.
- Title Bar HUD: Real-time FPS, UPS, and body count displayed via `sf::Window::setTitle`, updated every 0.5s. In-viewport HUD text removed.

## Review Focus

1. **Integrator swap during motion:** Swapping between Euler and RK4 mid-simulation preserves body positions, velocities, and IDs while switching numerical step logic immediately.
2. **Outside-click dismiss:** Clicking outside an open dropdown dismisses the dropdown and consumes the click without misfiring simulation events.
3. **Dynamic timestep adjustment:** Modifying `dt` at runtime updates the physics update accumulator threshold so the simulation seamlessly speeds up or slows down without freezing or spiral of death.
4. **Font-fallback resilience:** If no system font can be loaded, the app and title-bar HUD continue operating normally without throwing or crashing.
5. **Scenario reset clean state:** Resetting the scenario completely replaces all bodies and resets the simulation state without residual velocity or duplicate particle IDs.

---

### Task 1: PhysicsWorld Runtime Configuration (`set_integrator` & `set_dt`)

**Files:**
- Modify: `physics/include/physics_world.hpp`
- Modify: `physics/src/physics_world.cpp`
- Test: `tests/test_physics.cpp`

**Interfaces:**
- Consumes: `SimConfig`, `Body`, `IIntegrator`, `EulerIntegrator`, `RK4Integrator`
- Produces:
  - `void PhysicsWorld::set_integrator(const std::string& name);`
  - `const std::string& PhysicsWorld::integrator_name() const;`
  - `void PhysicsWorld::set_dt(float dt);`
  - `float PhysicsWorld::dt() const;`

- [ ] **Step 1: Write the failing tests in `tests/test_physics.cpp`**

Add tests to `tests/test_physics.cpp` for runtime integrator swapping and timestep updating:
```cpp
void test_runtime_integrator_and_dt_switch() {
    SimConfig cfg;
    cfg.integrator = "euler";
    cfg.dt = 0.001f;
    PhysicsWorld world(cfg);

    assert(world.integrator_name() == "euler");
    assert(world.dt() == 0.001f);

    std::vector<Body> bodies;
    bodies.push_back(Body{1, 1000.0f, {0.f, 0.f}, {0.f, 0.f}, 10.0f});
    bodies.push_back(Body{2, 1.0f, {100.f, 0.f}, {0.f, 5.0f}, 2.0f});
    world.set_bodies(bodies);

    // Swap integrator to rk4
    world.set_integrator("rk4");
    assert(world.integrator_name() == "rk4");
    assert(world.body_count() == 2);
    assert(world.bodies()[1].position.x == 100.f);

    // Step with RK4
    world.step();
    assert(world.bodies()[1].position.x != 100.f || world.bodies()[1].position.y != 0.f);

    // Update dt
    world.set_dt(0.005f);
    assert(world.dt() == 0.005f);

    // Swap back to euler
    world.set_integrator("euler");
    assert(world.integrator_name() == "euler");
    world.step();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `ctest --test-dir build -R test_physics --output-on-failure`
Expected: Compilation failure due to missing `set_integrator`, `integrator_name`, `set_dt`, `dt` methods.

- [ ] **Step 3: Implement methods in `physics/include/physics_world.hpp` and `physics/src/physics_world.cpp`**

In `physics/include/physics_world.hpp`:
```cpp
void set_integrator(const std::string& name);
const std::string& integrator_name() const;
void set_dt(float dt);
float dt() const;
```

In `physics/src/physics_world.cpp`:
- `set_integrator(const std::string& name)`: update `config_.integrator = name`, and reassign `integrator_` to `std::make_unique<RK4Integrator>()` if name == "rk4", else `std::make_unique<EulerIntegrator>()`.
- `integrator_name()`: return `config_.integrator`.
- `set_dt(float dt)`: set `config_.dt = dt`.
- `dt()`: return `config_.dt`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --target test_physics && ctest --test-dir build -R test_physics --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add physics/include/physics_world.hpp physics/src/physics_world.cpp tests/test_physics.cpp
git commit -m "feat(physics): add set_integrator and set_dt runtime mutators to PhysicsWorld"
```

---

### Task 2: Realistic Solar System Preset with Exponential Spacing

**Files:**
- Modify: `app/src/sim_presets.cpp`
- Test: `tests/test_presets.cpp`

**Interfaces:**
- Consumes: `presets::create_solar_system(const SimConfig&)`
- Produces: 6 bodies (Sun + Mercury, Venus, Earth, Mars, Jupiter) with realistic AU ratios, distinct masses/radii, and increasing gaps.

- [ ] **Step 1: Write the updated/failing tests in `tests/test_presets.cpp`**

In `tests/test_presets.cpp`, update `test_solar_system()` to assert:
- `bodies.size() == 6` (Sun + 5 planets: Mercury, Venus, Earth, Mars, Jupiter).
- Increasing distances from Sun: $d_1 < d_2 < d_3 < d_4 < d_5$.
- Increasing orbital gap jumps: outer jump $d_5 - d_4 > 100.0f$ while inner gap $d_2 - d_1 < 60.0f$.
- All planetary velocities satisfy circular orbit condition $v = \sqrt{G M / d}$ and perpendicularity $(r - r_{sun}) \cdot v = 0$.
- Distinct radii: Jupiter radius > Earth radius > Mercury radius.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target test_presets && ctest --test-dir build -R test_presets --output-on-failure`
Expected: FAIL (currently only 4 planets exist with uniform spacing).

- [ ] **Step 3: Implement realistic solar system spacing in `app/src/sim_presets.cpp`**

Update `presets::create_solar_system`:
```cpp
struct PlanetDef {
    float distance;
    float mass;
    float radius;
};

const PlanetDef planets[] = {
    {  50.0f, 0.5f, 2.5f}, // Mercury
    {  90.0f, 1.0f, 3.5f}, // Venus
    { 125.0f, 1.0f, 4.0f}, // Earth
    { 190.0f, 0.7f, 3.0f}, // Mars
    { 340.0f, 8.0f, 8.0f}  // Jupiter
};
```
Calculate circular orbital velocity for each planet: $v = \sqrt{G \cdot M_{sun} / d}$, positioned along the X-axis with velocity in $+Y$.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --target test_presets && ctest --test-dir build -R test_presets --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add app/src/sim_presets.cpp tests/test_presets.cpp
git commit -m "feat(presets): update solar system preset with realistic AU-ratio spacing"
```

---

### Task 3: Starfield Background Component

**Files:**
- Create: `app/include/starfield.hpp`
- Create: `app/src/starfield.cpp`
- Modify: `app/CMakeLists.txt`
- Create: `tests/test_starfield.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces:
  ```cpp
  class Starfield {
  public:
      Starfield(unsigned int width, unsigned int height, std::size_t count = 300, unsigned int seed = 1337);
      void render(sf::RenderTarget& target) const;
      std::size_t star_count() const;
      const std::vector<sf::Vector2f>& positions() const;
  };
  ```

- [ ] **Step 1: Write `tests/test_starfield.cpp` and update `tests/CMakeLists.txt`**

In `tests/test_starfield.cpp`:
```cpp
#include "starfield.hpp"
#include <cassert>
#include <iostream>

void test_starfield_generation() {
    Starfield stars(1280, 720, 250, 42);
    assert(stars.star_count() == 250);
    const auto& pos = stars.positions();
    assert(pos.size() == 250);

    for (const auto& p : pos) {
        assert(p.x >= 0.0f && p.x <= 1280.0f);
        assert(p.y >= 0.0f && p.y <= 720.0f);
    }

    // Deterministic with same seed
    Starfield stars2(1280, 720, 250, 42);
    for (std::size_t i = 0; i < 250; ++i) {
        assert(stars.positions()[i] == stars2.positions()[i]);
    }
}

int main() {
    test_starfield_generation();
    std::cout << "All starfield tests passed\n";
    return 0;
}
```

In `tests/CMakeLists.txt`:
```cmake
add_executable(test_starfield test_starfield.cpp ../app/src/starfield.cpp)
target_include_directories(test_starfield PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_starfield PRIVATE sfml-graphics sfml-window sfml-system)
add_test(NAME test_starfield COMMAND test_starfield)
```

- [ ] **Step 2: Run test to verify it fails to build**

Run: `cmake -B build && cmake --build build --target test_starfield`
Expected: Compilation failure because `starfield.hpp` does not exist yet.

- [ ] **Step 3: Implement `Starfield` in `app/include/starfield.hpp` and `app/src/starfield.cpp`**

In `app/include/starfield.hpp`: Define class `Starfield` with rendering support.
In `app/src/starfield.cpp`:
- Generate deterministic pseudo-random stars using `std::mt19937`.
- Store positions and alpha/brightness (values between 100 and 255 for depth).
- `render(sf::RenderTarget& target) const`: Draw stars using `sf::CircleShape` (radius 0.7f to 1.2f) or `sf::VertexArray` with `sf::PrimitiveType::Points` or small quads.
In `app/CMakeLists.txt`: Add `src/starfield.cpp` to `nbody-sim` sources.

- [ ] **Step 4: Build and run test to verify it passes**

Run: `cmake --build build --target test_starfield && ctest --test-dir build -R test_starfield --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add app/include/starfield.hpp app/src/starfield.cpp app/CMakeLists.txt tests/test_starfield.cpp tests/CMakeLists.txt
git commit -m "feat(ui): add background Starfield component and tests"
```

---

### Task 4: MenuBar & Dropdown UI Components

**Files:**
- Create: `app/include/menu_bar.hpp`
- Create: `app/src/menu_bar.cpp`
- Modify: `app/CMakeLists.txt`
- Create: `tests/test_menu_bar.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces:
  ```cpp
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
      bool is_any_dropdown_open() const;
      void close_dropdowns();
      const std::string& selected_scenario() const;
      const std::string& selected_integrator() const;
      float selected_dt() const;
      bool is_paused() const;
  };
  ```

- [ ] **Step 1: Write `tests/test_menu_bar.cpp` and update `tests/CMakeLists.txt`**

In `tests/test_menu_bar.cpp`:
Verify:
1. Initial state: scenario is "orbit", integrator is "euler", dt is 0.001f, paused is false. Dropdowns initially closed.
2. Clicking the "Scenario" item header opens the scenario dropdown (`is_any_dropdown_open() == true`).
3. Clicking on an option inside the dropdown (e.g. "solar") invokes callback, updates selected scenario, and closes dropdown.
4. Clicking outside dropdown dismisses dropdown and returns `true` (event consumed).
5. Clicking "Pause" button invokes pause toggle callback.
6. Hover detection via `handle_mouse_moved`.

In `tests/CMakeLists.txt`:
```cmake
add_executable(test_menu_bar test_menu_bar.cpp ../app/src/menu_bar.cpp)
target_include_directories(test_menu_bar PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_menu_bar PRIVATE sfml-graphics sfml-window sfml-system)
add_test(NAME test_menu_bar COMMAND test_menu_bar)
```

- [ ] **Step 2: Run test to verify it fails to build**

Run: `cmake -B build && cmake --build build --target test_menu_bar`
Expected: Compilation failure because `menu_bar.hpp` does not exist yet.

- [ ] **Step 3: Implement `MenuBar` in `app/include/menu_bar.hpp` and `app/src/menu_bar.cpp`**

In `app/include/menu_bar.hpp` and `app/src/menu_bar.cpp`:
- Implement horizontal bar rendering (`height = 35.0f`, dark background `sf::Color(30, 30, 30, 235)`).
- Items:
  - "Scenario" (at x ≈ 10): options ["Orbit", "Solar System", "Random Cloud"], values ["orbit", "solar", "random"].
  - "Integrator" (at x ≈ 160): options ["Euler", "RK4"], values ["euler", "rk4"].
  - "Timestep" (at x ≈ 300): options ["0.0001", "0.0005", "0.001", "0.005", "0.01"], values [0.0001f, 0.0005f, 0.001f, 0.005f, 0.01f].
  - "Pause/Resume" button (right aligned at `width - 120.0f`).
- Handle mouse hover and clicks with bounding box hit-testing (`sf::FloatRect::contains`).
- Draw open dropdowns beneath the corresponding menu item with highlight on hovered rows and active selection indicator.
In `app/CMakeLists.txt`: Add `src/menu_bar.cpp` to `nbody-sim` sources.

- [ ] **Step 4: Build and run test to verify it passes**

Run: `cmake --build build --target test_menu_bar && ctest --test-dir build -R test_menu_bar --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add app/include/menu_bar.hpp app/src/menu_bar.cpp app/CMakeLists.txt tests/test_menu_bar.cpp tests/CMakeLists.txt
git commit -m "feat(ui): implement MenuBar and Dropdown UI components with hit-testing tests"
```

---

### Task 5: SimulationApp Integration (Top Bar, Viewport Offset, Title Bar HUD)

**Files:**
- Modify: `app/include/simulation_app.hpp`
- Modify: `app/src/simulation_app.cpp`

**Interfaces:**
- Consumes: `Starfield`, `MenuBar`, `PhysicsWorld`, `SimConfig`, `presets`
- Modifies:
  - `SimulationApp::handle_events()` routes mouse events to `MenuBar`
  - `SimulationApp::world_to_screen()` adds `menu_bar_.height()` vertical offset
  - Replaces `draw_hud()` with periodic `window_.setTitle(...)` every 0.5s
  - Connects menu callbacks to reset presets, swap integrators, update `dt`, and toggle pause

- [ ] **Step 1: Update `SimulationApp` header and implementation**

In `app/include/simulation_app.hpp`:
- Add `Starfield starfield_;` and `MenuBar menu_bar_;` members.
- Add `void reset_preset(const std::string& preset_name);`
- Add `void update_title_bar(sf::Time elapsed);`
- Remove `void draw_hud();`
- Update `world_to_screen` declaration.

In `app/src/simulation_app.cpp`:
- Initialize `starfield_` and `menu_bar_` in constructor.
- Set up `menu_bar_` callbacks:
  - `on_scenario_changed`: calls `reset_preset(name)` which fetches bodies from `presets::` and calls `world_.set_bodies(bodies)`
  - `on_integrator_changed`: calls `world_.set_integrator(name)`
  - `on_timestep_changed`: calls `world_.set_dt(dt)`
  - `on_pause_toggled`: toggles `paused_` and syncs `menu_bar_.set_paused(paused_)`
- In `handle_events()`:
  - Check `sf::Event::MouseButtonPressed`: if `menu_bar_.handle_mouse_pressed(pos, btn)` returns true, consume event.
  - Check `sf::Event::MouseMoved`: pass `menu_bar_.handle_mouse_moved(pos)`.
  - On KeyPressed Space: toggle `paused_` and sync `menu_bar_.set_paused(paused_)`.
- In `world_to_screen(sf::Vector2f world_pos)`:
  - `return {world_pos.x, world_pos.y + menu_bar_.height()};`
- In `update_title_bar(sf::Time elapsed)`:
  - Every 0.5s, format: `"N-Body Sim | FPS: " + to_string(fps_display_) + " | UPS: " + to_string(ups_display_) + " | Bodies: " + to_string(world_.body_count()) + (paused_ ? " [PAUSED]" : "")`.
  - Set via `window_.setTitle(title)`.
- In `run()`:
  - Compute `physics_dt = sf::seconds(world_.dt());` inside the loop so runtime changes take effect immediately.
  - Call `update_title_bar(frame_time)`.
- In `render()`:
  - `window_.clear(sf::Color::Black);`
  - `starfield_.render(window_);`
  - Render bodies with `world_to_screen`.
  - `menu_bar_.render(window_);`
  - `window_.display();`

- [ ] **Step 2: Build project and verify compilation**

Run: `cmake --build build`
Expected: Build succeeds with 0 warnings.

- [ ] **Step 3: Run full test suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: 6/6 tests pass (test_body, test_integrators, test_physics, test_presets, test_starfield, test_menu_bar).

- [ ] **Step 4: Commit**

```bash
git add app/include/simulation_app.hpp app/src/simulation_app.cpp
git commit -m "feat(app): integrate MenuBar, Starfield, vertical viewport offset, and title bar HUD"
```

---

### Task 6: Headless Verification & Smoke Test

**Files:**
- Modify: `tests/CMakeLists.txt`
- Create: `tests/test_simulation_app.cpp`

**Interfaces:**
- Verifies full integration: app initialization, preset switching, integrator switching, timestep update, window title generation, and headless step without crash.

- [ ] **Step 1: Write integration smoke test `tests/test_simulation_app.cpp`**

In `tests/test_simulation_app.cpp`:
```cpp
#include "simulation_app.hpp"
#include <cassert>
#include <iostream>

void test_app_initialization_and_reset() {
    SimConfig cfg;
    cfg.window_width = 800;
    cfg.window_height = 600;
    cfg.preset = "orbit";
    cfg.integrator = "euler";
    cfg.dt = 0.001f;

    SimulationApp app(cfg);
    assert(app.world().body_count() == 2);
    assert(!app.is_paused());

    // Test scenario resetting
    app.reset_preset("solar");
    assert(app.world().body_count() == 6);

    app.reset_preset("random");
    assert(app.world().body_count() >= 50);

    // Test integrator swapping on app world
    app.world_mut().set_integrator("rk4");
    assert(app.world().integrator_name() == "rk4");

    // Test timestep updating
    app.world_mut().set_dt(0.005f);
    assert(app.world().dt() == 0.005f);
}

int main() {
    test_app_initialization_and_reset();
    std::cout << "All simulation app integration tests passed\n";
    return 0;
}
```

- [ ] **Step 2: Add executable and test to `tests/CMakeLists.txt`**

```cmake
add_executable(test_simulation_app
    test_simulation_app.cpp
    ../app/src/simulation_app.cpp
    ../app/src/sim_presets.cpp
    ../app/src/starfield.cpp
    ../app/src/menu_bar.cpp
)
target_include_directories(test_simulation_app PRIVATE ${CMAKE_SOURCE_DIR}/app/include)
target_link_libraries(test_simulation_app PRIVATE
    nbody-physics
    sfml-graphics
    sfml-window
    sfml-system
)
add_test(NAME test_simulation_app COMMAND test_simulation_app)
```

- [ ] **Step 3: Run ctest under xvfb**

Run: `xvfb-run -a ctest --test-dir build --output-on-failure`
Expected: 7/7 tests pass.

- [ ] **Step 4: Commit**

```bash
git add tests/test_simulation_app.cpp tests/CMakeLists.txt
git commit -m "test(app): add headless simulation app integration test"
```
