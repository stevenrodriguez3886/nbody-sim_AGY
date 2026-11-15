# Simulation Menu & Visual Polish — Design Document

**Date:** 2026-11-14
**Status:** Draft

## Intent & Why

The simulator currently requires CLI arguments to choose a scenario or integrator, and there's no way to change settings once it's running. The user wants to experiment interactively — tweak timestep, swap integrators, and switch scenarios while watching particles move — without restarting. The black background also feels lifeless.

## Goals

1. An always-visible sidebar control panel with mouse-clickable widgets for runtime configuration.
2. A static background starfield across all simulations for visual depth.
3. Realistic solar system preset with exponential orbital spacing.
4. HUD info (FPS, UPS, body count) moved to the window title bar; in-viewport HUD text removed.

## Non-Goals

- No editable G or epsilon in the UI (implementation details, not user-facing knobs).
- No drag-and-drop body placement or mouse interaction with individual particles.
- No third-party GUI library — everything built with SFML 3 primitives.

## Anti-Goals (would make this a failure)

- A menu that pauses or blocks the simulation while visible.
- Controls that require keyboard shortcuts to discover — everything must be visually obvious and clickable.
- Breaking the existing physics library's independence from SFML graphics.

## Constraints

- C++17, SFML 3.0 only (no ImGui, no third-party widget libs).
- All UI/menu code lives in `app/` — the `physics/` library must not gain any SFML Graphics dependency.
- Zero compiler warnings under `-Wall -Wextra -Wpedantic`.
- All existing tests must continue to pass.

---

## Design

### 1. Top Menu Bar with Dropdowns

**Layout:** A horizontal bar across the **top** of the window. Height: ~35px. Background: solid dark rectangle (`sf::Color(30, 30, 30, 230)`).

The simulation viewport starts below the menu bar (`y_offset = menu_bar_height`). The full window width is preserved for the simulation. The `world_to_screen` coordinate mapping adds the vertical offset so bodies aren't drawn behind the bar.

**Menu bar items (left to right):** "Scenario", "Integrator", "Timestep", and a "Pause" / "Resume" button on the far right.

**Widget system** — built from `sf::RectangleShape` + `sf::Text`:

#### Menu Bar Item
- A clickable text label in the top bar.
- Two visual states: normal (text only) and open (highlighted background, dropdown visible below).
- Clicking toggles the dropdown open/closed. Clicking a different menu item closes the previous dropdown and opens the new one. Clicking anywhere outside an open dropdown closes it.

#### Dropdown Menu
- A vertical list of options that appears below its parent menu bar item when opened.
- Background: solid dark rectangle matching the menu bar.
- Each option is a row with hover highlighting.
- The currently selected/active option has a checkmark or accent color indicator.
- Clicking an option triggers its action and closes the dropdown.

#### Dropdown Options per Menu:

**Scenario dropdown:**
- "Orbit" / "Solar System" / "Random Cloud"
- Active preset has a visual indicator (accent color or checkmark).
- Clicking a different preset resets the simulation with that preset's bodies.

**Integrator dropdown:**
- "Euler" / "RK4"
- Active integrator has a visual indicator.
- Switching takes effect immediately on the next physics step (no reset).

**Timestep dropdown:**
- A list of preset timestep values: 0.0001, 0.0005, 0.001, 0.005, 0.01
- Active value has a visual indicator.
- Selecting a value updates `dt` immediately.

**Pause/Resume button:**
- A simple button on the right end of the menu bar (not a dropdown).
- Mirrors the Space key behavior. Label toggles between "⏸ Pause" and "▶ Resume".

**Mouse event routing:** `SimulationApp::handle_events()` checks mouse clicks against menu bar and any open dropdown bounds first. If the click is inside a menu widget, it's consumed. If an open dropdown exists and the click is outside it (and outside the menu bar), the dropdown closes. Clicks in the simulation area below the menu bar are ignored (reserved for future interaction).

### 2. Background Starfield

A fixed set of small dots (200–400 points) scattered across the full window area, generated once at startup (or on window resize). Stored as a `std::vector<sf::Vector2f>` of positions with associated sizes/brightness.

**Rendering:** Draw before bodies, after clearing the window. Each star is a small `sf::CircleShape` (radius 0.5–1.5px) in white or light gray with slight alpha variation for depth. Since the starfield is static and purely cosmetic, it can also be rendered as a `sf::VertexArray` of points for efficiency — builder's choice on which is simpler.

The starfield covers the full window including behind the sidebar (the sidebar's semi-transparent background will dim the stars behind it naturally).

### 3. Solar System Preset Fix

Current distances: 80, 150, 240, 340 (roughly uniform ~70-100px gaps).

**New distances** based on realistic ratios (using Earth = 1.0 AU as reference, scaled to fit the viewport):

| Planet   | Real AU | Ratio | Sim Distance (px) |
|----------|---------|-------|--------------------|
| Mercury  | 0.39    | 0.39  | ~50                |
| Venus    | 0.72    | 0.72  | ~90                |
| Earth    | 1.00    | 1.00  | ~125               |
| Mars     | 1.52    | 1.52  | ~190               |
| Jupiter  | 5.20    | 5.20  | ~350 (if fits)     |

The exact pixel distances should be scaled so the outermost planet fits comfortably within the sim viewport (accounting for the sidebar). The builder should calculate: `max_distance = (effective_viewport_width / 2) * 0.85` and scale the AU ratios to fit within that.

Masses and radii should also have more variation to look distinct (Jupiter much larger than Mercury).

### 4. HUD to Title Bar

Replace the current `draw_hud()` rendered text with a periodic `window_.setTitle(...)` call. Update every ~0.5 seconds (not every frame) to avoid excessive string allocations.

Format: `"N-Body Sim | FPS: 60 | UPS: 1000 | Bodies: 5 | Paused"` (omit "Paused" when running).

The `sf::Font` member and `font_loaded_` flag can be kept since the sidebar widgets need the font for button labels. The `draw_hud()` method is removed.

### 5. PhysicsWorld Reset & Integrator Swap

`PhysicsWorld` currently has no way to replace its bodies or integrator after construction. Two runtime mutations are needed:

**Scenario reset (replaces bodies + may change integrator):** The app reconstructs the `PhysicsWorld` entirely — either hold it via `std::unique_ptr` or make it movable. Reconstruction is simpler and avoids partial-reset bugs. *(Builder's choice on the mechanism.)*

**Integrator swap without reset (bodies keep current state):** Add a `set_integrator(const std::string&)` method to `PhysicsWorld` that reconstructs the internal `integrator_` unique_ptr without touching the body list. This is a minimal API addition that keeps physics independent and satisfies the "no reset on integrator change" requirement. *(My call — the builder check flagged this gap.)*

### 6. Coordinate Mapping & Menu Bar Viewport

Currently `world_to_screen` is an identity function and presets place bodies using raw pixel coordinates centered on the full window (`window_width / 2`, `window_height / 2`).

**Approach:** Since the menu bar only takes ~35px from the top, the impact is minimal. `world_to_screen` adds a vertical offset (`menu_bar_height`) to all positions so bodies render below the bar. Presets continue using the full `window_width` for horizontal centering. Vertical centering can use `window_height / 2` — the slight upward shift (~17px) from the menu bar is negligible. *(My call — much simpler than the sidebar approach the builder check originally flagged.)*

### 7. Keyboard Shortcuts

Existing shortcuts stay:
- **Space** — pause/resume (also reflected in the sidebar Pause button state)
- **Escape** — quit

No new keyboard shortcuts needed — everything is in the sidebar.

---

## What's Left to the Builder

- Exact pixel positions and spacing of sidebar widgets (as long as they fit in ~220px width and don't overlap).
- Whether to use `sf::VertexArray` or individual `sf::CircleShape` for the starfield.
- Whether to reconstruct `PhysicsWorld` or add a `reset()` method.
- Exact colors for button states (normal/hover/selected) — just make them visually distinct.
- Font size for sidebar labels and buttons.
