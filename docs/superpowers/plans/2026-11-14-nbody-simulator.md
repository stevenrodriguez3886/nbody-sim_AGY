# N-Body Gravity Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a real-time N-body gravitational simulator with pluggable integrators (Euler, RK4), inelastic collision merging, configurable presets, and a fixed-timestep physics loop rendered via SFML 3.0.

**Architecture:** Array-of-Structures body storage feeds an all-pairs O(N²) gravity solver. A polymorphic `IIntegrator` interface decouples the numerical scheme from the physics world. `SimulationApp` owns the SFML window and runs a fixed-timestep accumulator loop that steps the `PhysicsWorld` at a deterministic rate independent of the render frame rate.

**Tech Stack:** C++17, CMake ≥ 3.22 (FetchContent for SFML 3.0.0), SFML 3 (`sfml-graphics`, `sfml-window`, `sfml-system`).

**Spec:** The user's request message in this conversation serves as the spec.

## Global Constraints

- C++17 standard required (`CMAKE_CXX_STANDARD 17`, `CMAKE_CXX_STANDARD_REQUIRED ON`).
- Compiler warnings: `-Wall -Wextra -Wpedantic` on GCC/Clang.
- SFML 3.0.0 fetched via `FetchContent` — no system-installed SFML assumed.
- All SFML API calls must use SFML 3 patterns: `std::optional` event polling, scoped enums (`sf::Keyboard::Key::Space`), `sf::Font` as first parameter to `sf::Text`, `sf::VideoMode({w, h})` brace syntax.
- Floating-point type: `float` throughout (positions, velocities, masses). `sf::Vector2f` for 2D vectors.
- No third-party libraries beyond SFML 3 and the C++ standard library.
- All physics constants (G, epsilon, dt) defined in a single `SimConfig` struct.

## Review Focus

1. **Two bodies at zero relative velocity on top of each other** — the softening parameter `ε` must prevent `Inf`/`NaN` forces; verify `ε² > 0` is enforced at construction.
2. **Single remaining body after all merges** — the simulation must not crash or divide by zero when N = 1 (no pairwise forces to compute).
3. **Extremely large mass ratios (1e10 : 1)** — Euler will diverge; RK4 should remain stable for moderate `dt`. No assertion or overflow should trigger.
4. **Zero-mass body** — gravitational formula produces zero force correctly; body must not be excluded from collision checks.
5. **Window resize during simulation** — coordinate transform must recompute from current window size, not a cached value. Verify shapes stay within view.

---

## File Structure

```
nbody-sim_AGY/
├── CMakeLists.txt
├── include/
│   ├── body.hpp
│   ├── physics_world.hpp
│   ├── integrator.hpp
│   ├── euler_integrator.hpp
│   ├── rk4_integrator.hpp
│   ├── sim_config.hpp
│   ├── sim_presets.hpp
│   └── simulation_app.hpp
├── src/
│   ├── main.cpp
│   ├── body.cpp
│   ├── physics_world.cpp
│   ├── euler_integrator.cpp
│   ├── rk4_integrator.cpp
│   ├── sim_presets.cpp
│   └── simulation_app.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── test_body.cpp
│   ├── test_physics.cpp
│   ├── test_integrators.cpp
│   └── test_presets.cpp
└── docs/
    └── superpowers/
        └── plans/
            └── 2026-11-14-nbody-simulator.md  (this file)
```

**Responsibilities:**
- `sim_config.hpp` — `SimConfig` struct holding all tunables (G, ε, dt, window size, integrator choice, preset choice).
- `body.hpp / .cpp` — `Body` struct and its merge operation.
- `integrator.hpp` — Abstract `IIntegrator` interface.
- `euler_integrator.hpp / .cpp` — Explicit Euler implementation.
- `rk4_integrator.hpp / .cpp` — Classical RK4 implementation.
- `physics_world.hpp / .cpp` — Owns `std::vector<Body>`, computes pairwise forces, delegates to integrator, handles collisions.
- `sim_presets.hpp / .cpp` — Factory functions that return `std::vector<Body>` for each preset scenario.
- `simulation_app.hpp / .cpp` — SFML window, event loop, fixed-timestep accumulator, HUD rendering, coordinate transforms.
- `main.cpp` — Parses command-line args, constructs `SimulationApp`, calls `run()`.
- `tests/` — Lightweight test harness (no framework; each `test_*.cpp` is a separate executable with `assert` macros returning 0 on success, non-zero on failure).

---

## Mathematical Specifications

### Gravitational Force (Pairwise)

For bodies $i$ and $j$:

$$\vec{r}_{ij} = \vec{x}_j - \vec{x}_i$$

$$F_{ij} = G \frac{m_i \, m_j}{\left( |\vec{r}_{ij}|^2 + \varepsilon^2 \right)^{3/2}} \cdot \vec{r}_{ij}$$

Note: the denominator uses the **3/2 power** of $(r^2 + \varepsilon^2)$, not $r^2 + \varepsilon^2$ alone. This is because the force direction requires dividing the unit vector $\hat{r}$ by $r$, and the magnitude uses $1/r^2$, combining to $r / r^3 = 1/r^2$ in direction. With softening: $\vec{r} / (r^2 + \varepsilon^2)^{3/2}$.

Acceleration on body $i$:

$$\vec{a}_i = \sum_{j \neq i} G \frac{m_j}{\left( |\vec{r}_{ij}|^2 + \varepsilon^2 \right)^{3/2}} \cdot \vec{r}_{ij}$$

### State Vector Formulation

For body $i$, the state is $y_i = [\vec{x}_i, \vec{v}_i]^T$. The derivative:

$$f(t, y_i) = \left[\vec{v}_i, \; \vec{a}_i(\vec{x}_1, \ldots, \vec{x}_N)\right]^T$$

The system state is the collection of all $N$ body states.

### Explicit Euler

For each body $i$, given timestep $h$:

$$\vec{v}_i^{n+1} = \vec{v}_i^n + h \cdot \vec{a}_i^n$$
$$\vec{x}_i^{n+1} = \vec{x}_i^n + h \cdot \vec{v}_i^n$$

Note: position uses the **old** velocity (not the updated one). This is the standard explicit (forward) Euler.

### RK4 (4th-Order Runge-Kutta)

Given system state $Y^n$ (all positions and velocities), timestep $h$:

Let `compute_accelerations(positions) → accelerations` be a function that takes all N positions and returns all N acceleration vectors.

For each body $i$, let $x = \vec{x}_i^n$, $v = \vec{v}_i^n$:

**Stage k1:**
- $k1_v = \vec{a}_i($ all current positions $)$
- $k1_x = v$

**Stage k2** (evaluate at $t + h/2$ with k1-shifted state):
- Compute positions: $x'_j = x_j + (h/2) \cdot k1_{x,j}$ for all $j$
- $k2_v = \vec{a}_i($ shifted positions $)$
- $k2_x = v + (h/2) \cdot k1_v$

**Stage k3** (evaluate at $t + h/2$ with k2-shifted state):
- Compute positions: $x'_j = x_j + (h/2) \cdot k2_{x,j}$ for all $j$
- $k3_v = \vec{a}_i($ shifted positions $)$
- $k3_x = v + (h/2) \cdot k2_v$

**Stage k4** (evaluate at $t + h$ with k3-shifted state):
- Compute positions: $x'_j = x_j + h \cdot k3_{x,j}$ for all $j$
- $k4_v = \vec{a}_i($ shifted positions $)$
- $k4_x = v + h \cdot k3_v$

**Update:**

$$\vec{x}_i^{n+1} = \vec{x}_i^n + \frac{h}{6}(k1_x + 2 k2_x + 2 k3_x + k4_x)$$
$$\vec{v}_i^{n+1} = \vec{v}_i^n + \frac{h}{6}(k1_v + 2 k2_v + 2 k3_v + k4_v)$$

**Critical implementation note:** Each RK4 stage must recompute accelerations for **all bodies at once** using the stage-shifted positions. You cannot compute per-body stages independently — the accelerations are coupled.

### Inelastic Merge Collision

Two bodies $i$ and $j$ collide when:

$$|\vec{r}_{ij}| < r_i + r_j$$

where $r_i$, $r_j$ are visual radii. On collision, merge into one body:

- **Mass:** $m' = m_i + m_j$
- **Position (center of mass):** $\vec{x}' = \frac{m_i \vec{x}_i + m_j \vec{x}_j}{m_i + m_j}$
- **Velocity (momentum conservation):** $\vec{v}' = \frac{m_i \vec{v}_i + m_j \vec{v}_j}{m_i + m_j}$
- **Radius (volume conservation):** $r' = \sqrt{r_i^2 + r_j^2}$  (2D area conservation: $\pi r'^2 = \pi r_i^2 + \pi r_j^2$)
- **ID:** Keep the ID of the more massive body (or the lower ID on tie).

Collision detection runs **after** each integration step. Process collisions iteratively: after merging, re-check the updated list until no more collisions remain in that timestep.

---

## Task 1: CMake Build System and Project Skeleton

**Files:**
- Create: `CMakeLists.txt`
- Create: `src/main.cpp`
- Create: `include/sim_config.hpp`
- Create: `tests/CMakeLists.txt`
- Create: `tests/test_body.cpp` (placeholder)

**Interfaces:**
- Consumes: nothing (first task)
- Produces: A buildable CMake project. `SimConfig` struct used by all later tasks:
  ```cpp
  struct SimConfig {
      float G = 1.0f;
      float epsilon = 5.0f;
      float dt = 0.001f;
      unsigned int window_width = 1280;
      unsigned int window_height = 720;
      std::string integrator = "euler";   // "euler" or "rk4"
      std::string preset = "orbit";       // "orbit", "solar", "random"
      uint64_t next_id = 1;
  };
  ```

- [ ] **Step 1: Create `CMakeLists.txt`**

  Top-level CMake file. Requirements:
  - `cmake_minimum_required(VERSION 3.22)`
  - Project name `nbody-sim`, language `CXX`
  - `CMAKE_CXX_STANDARD 17`, `CMAKE_CXX_STANDARD_REQUIRED ON`
  - `FetchContent_Declare` for SFML with `GIT_REPOSITORY https://github.com/SFML/SFML.git`, `GIT_TAG 3.0.0`, `GIT_SHALLOW ON`
  - Set `BUILD_SHARED_LIBS OFF` before `FetchContent_MakeAvailable(SFML)`
  - `add_executable(nbody-sim src/main.cpp)`
  - `target_include_directories(nbody-sim PRIVATE ${CMAKE_SOURCE_DIR}/include)`
  - `target_link_libraries(nbody-sim PRIVATE sfml-graphics sfml-window sfml-system)`
  - `target_compile_options(nbody-sim PRIVATE -Wall -Wextra -Wpedantic)` guarded by `if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")`
  - `add_subdirectory(tests)`

- [ ] **Step 2: Create `include/sim_config.hpp`**

  Header-only struct as specified in Produces above. Include `<string>` and `<cstdint>`. Use `#pragma once`.

- [ ] **Step 3: Create `src/main.cpp`**

  Minimal main that constructs a `SimConfig`, prints "N-body simulator starting..." to stdout, and returns 0. Include `sim_config.hpp` and `<iostream>`.

- [ ] **Step 4: Create `tests/CMakeLists.txt`**

  Placeholder that defines no targets yet. Add a comment `# Test targets added by later tasks`.

- [ ] **Step 5: Build and verify**

  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Debug
  cmake --build build
  ./build/nbody-sim
  ```
  Expected: prints "N-body simulator starting..." and exits 0. SFML fetches and links successfully.

- [ ] **Step 6: Commit**

  ```bash
  git init
  echo "build/" > .gitignore
  git add CMakeLists.txt include/ src/ tests/ .gitignore docs/
  git commit -m "feat: cmake skeleton with SFML 3 FetchContent and SimConfig"
  ```

---

## Task 2: Body Struct and Merge Operation

**Files:**
- Create: `include/body.hpp`
- Create: `src/body.cpp`
- Create: `tests/test_body.cpp` (replace placeholder)
- Modify: `CMakeLists.txt` — add `src/body.cpp` to `nbody-sim` sources
- Modify: `tests/CMakeLists.txt` — add `test_body` executable

**Interfaces:**
- Consumes: `SimConfig::next_id` for ID assignment
- Produces: `Body` struct and free function `merge`:
  ```cpp
  struct Body {
      uint64_t id;
      float mass;
      sf::Vector2f position;
      sf::Vector2f velocity;
      float radius;
  };
  Body merge(const Body& a, const Body& b);
  bool check_collision(const Body& a, const Body& b);
  ```

- [ ] **Step 1: Write failing tests in `tests/test_body.cpp`**

  Three test functions using `<cassert>`:
  ```cpp
  void test_merge_mass_and_momentum() {
      Body a{1, 10.0f, {0.f, 0.f}, {1.f, 0.f}, 5.f};
      Body b{2, 20.0f, {10.f, 0.f}, {-1.f, 0.f}, 3.f};
      Body m = merge(a, b);
      assert(m.mass == 30.0f);
      // CoM position: (10*0 + 20*10)/30 = 200/30 ≈ 6.6667
      assert(std::abs(m.position.x - 6.6667f) < 0.01f);
      // Momentum-conserving velocity: (10*1 + 20*(-1))/30 = -10/30 ≈ -0.3333
      assert(std::abs(m.velocity.x - (-0.3333f)) < 0.01f);
      // Area-conserving radius: sqrt(25 + 9) = sqrt(34) ≈ 5.831
      assert(std::abs(m.radius - std::sqrt(34.f)) < 0.01f);
      // ID of more massive body
      assert(m.id == 2);
  }
  void test_collision_detection() {
      Body a{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 5.f};
      Body b{2, 1.f, {8.f, 0.f}, {0.f, 0.f}, 5.f};
      assert(check_collision(a, b) == true);   // distance 8 < 5+5
      Body c{3, 1.f, {20.f, 0.f}, {0.f, 0.f}, 5.f};
      assert(check_collision(a, c) == false);  // distance 20 > 5+5
  }
  void test_merge_equal_mass_preserves_id() {
      Body a{1, 10.0f, {0.f, 0.f}, {0.f, 0.f}, 5.f};
      Body b{2, 10.0f, {1.f, 0.f}, {0.f, 0.f}, 5.f};
      Body m = merge(a, b);
      assert(m.id == 1);  // equal mass → lower ID wins
  }
  ```
  `main()` calls all three, prints "All body tests passed", returns 0.

- [ ] **Step 2: Run tests to verify they fail**

  ```bash
  cmake --build build
  ./build/tests/test_body
  ```
  Expected: linker error (merge/check_collision not defined).

- [ ] **Step 3: Implement `Body`, `merge`, `check_collision`**

  In `include/body.hpp`: define the `Body` struct and declare the two free functions. In `src/body.cpp`: implement them per the merge math in the Mathematical Specifications section. Use `std::hypot` or manual `std::sqrt(dx*dx + dy*dy)` for distance in `check_collision`.

- [ ] **Step 4: Update `CMakeLists.txt`**

  Add `src/body.cpp` to the `nbody-sim` target's source list. In `tests/CMakeLists.txt`, add:
  ```cmake
  add_executable(test_body test_body.cpp ../src/body.cpp)
  target_include_directories(test_body PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_link_libraries(test_body PRIVATE sfml-system)
  ```

- [ ] **Step 5: Run tests to verify they pass**

  ```bash
  cmake --build build
  ./build/tests/test_body
  ```
  Expected: "All body tests passed", exit 0.

- [ ] **Step 6: Commit**

  ```bash
  git add include/body.hpp src/body.cpp tests/test_body.cpp tests/CMakeLists.txt CMakeLists.txt
  git commit -m "feat: Body struct with merge and collision detection"
  ```

---

## Task 3: Integrator Interface and Explicit Euler

**Files:**
- Create: `include/integrator.hpp`
- Create: `include/euler_integrator.hpp`
- Create: `src/euler_integrator.cpp`
- Create: `tests/test_integrators.cpp`
- Modify: `CMakeLists.txt` — add `src/euler_integrator.cpp`
- Modify: `tests/CMakeLists.txt` — add `test_integrators`

**Interfaces:**
- Consumes: `Body` struct from Task 2
- Produces: `IIntegrator` interface and `EulerIntegrator`:
  ```cpp
  // Callback type: given a vector of positions, return vector of accelerations
  using AccelerationFn = std::function<std::vector<sf::Vector2f>(
      const std::vector<sf::Vector2f>& positions,
      const std::vector<float>& masses)>;

  class IIntegrator {
  public:
      virtual ~IIntegrator() = default;
      virtual void step(std::vector<Body>& bodies, float dt,
                        const AccelerationFn& accel_fn) = 0;
  };

  class EulerIntegrator : public IIntegrator {
  public:
      void step(std::vector<Body>& bodies, float dt,
                const AccelerationFn& accel_fn) override;
  };
  ```

- [ ] **Step 1: Write failing tests in `tests/test_integrators.cpp`**

  Test with a trivial constant-acceleration scenario (no gravity, just a fixed acceleration):
  ```cpp
  void test_euler_constant_accel() {
      // Single body at origin, v=(0,0), constant accel=(1,0)
      std::vector<Body> bodies = {{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 1.f}};
      AccelerationFn constant_accel = [](const auto&, const auto&) {
          return std::vector<sf::Vector2f>{{1.f, 0.f}};
      };
      EulerIntegrator euler;
      euler.step(bodies, 1.0f, constant_accel);
      // After 1 step of dt=1: v = 0 + 1*1 = 1, x = 0 + 1*0 = 0 (uses old v)
      assert(std::abs(bodies[0].velocity.x - 1.0f) < 1e-5f);
      assert(std::abs(bodies[0].position.x - 0.0f) < 1e-5f);
      euler.step(bodies, 1.0f, constant_accel);
      // After 2nd step: v = 1 + 1 = 2, x = 0 + 1*1 = 1
      assert(std::abs(bodies[0].velocity.x - 2.0f) < 1e-5f);
      assert(std::abs(bodies[0].position.x - 1.0f) < 1e-5f);
  }
  ```

- [ ] **Step 2: Run test to verify it fails**

  Expected: linker error (EulerIntegrator not defined).

- [ ] **Step 3: Implement `IIntegrator` in `include/integrator.hpp`**

  Pure abstract class as specified in Produces. Include `<vector>`, `<functional>`, `body.hpp`, `<SFML/System/Vector2.hpp>`.

- [ ] **Step 4: Implement `EulerIntegrator`**

  In `include/euler_integrator.hpp`: declare the class inheriting `IIntegrator`. In `src/euler_integrator.cpp`: implement `step()`. Extract all positions and masses from bodies, call `accel_fn`, then for each body: `v += dt * a`, `x += dt * v_old` (compute both updates before writing position, or save old velocity first).

- [ ] **Step 5: Update build files, run tests**

  ```bash
  cmake --build build
  ./build/tests/test_integrators
  ```
  Expected: "All integrator tests passed", exit 0.

- [ ] **Step 6: Commit**

  ```bash
  git add include/integrator.hpp include/euler_integrator.hpp src/euler_integrator.cpp tests/test_integrators.cpp CMakeLists.txt tests/CMakeLists.txt
  git commit -m "feat: IIntegrator interface and EulerIntegrator"
  ```

---

## Task 4: RK4 Integrator

**Files:**
- Create: `include/rk4_integrator.hpp`
- Create: `src/rk4_integrator.cpp`
- Modify: `tests/test_integrators.cpp` — add RK4 test
- Modify: `CMakeLists.txt` — add `src/rk4_integrator.cpp`

**Interfaces:**
- Consumes: `IIntegrator` from Task 3, `AccelerationFn` type from Task 3
- Produces:
  ```cpp
  class RK4Integrator : public IIntegrator {
  public:
      void step(std::vector<Body>& bodies, float dt,
                const AccelerationFn& accel_fn) override;
  };
  ```

- [ ] **Step 1: Add RK4 test to `tests/test_integrators.cpp`**

  Test RK4 accuracy against an analytical solution. A single body under constant acceleration should produce **exact** results with RK4 (constant accel is a polynomial of degree 1 in time; RK4 is exact for polynomials up to degree 4):
  ```cpp
  void test_rk4_constant_accel() {
      std::vector<Body> bodies = {{1, 1.f, {0.f, 0.f}, {0.f, 0.f}, 1.f}};
      AccelerationFn constant_accel = [](const auto&, const auto&) {
          return std::vector<sf::Vector2f>{{1.f, 0.f}};
      };
      RK4Integrator rk4;
      // 10 steps of dt=0.1 → t=1.0
      for (int i = 0; i < 10; ++i)
          rk4.step(bodies, 0.1f, constant_accel);
      // Analytical: x = 0.5 * a * t^2 = 0.5, v = a*t = 1.0
      assert(std::abs(bodies[0].position.x - 0.5f) < 1e-4f);
      assert(std::abs(bodies[0].velocity.x - 1.0f) < 1e-4f);
  }
  ```

  Also add a convergence test: compare RK4 vs Euler for a simple harmonic oscillator–like setup (acceleration proportional to negative displacement). RK4 with small dt should produce significantly lower error than Euler:
  ```cpp
  void test_rk4_more_accurate_than_euler() {
      // Spring-like: a = -x (omega=1, period=2*pi)
      // Start at x=1, v=0. After t=pi/2, analytical: x≈0, v≈-1
      auto spring_accel = [](const std::vector<sf::Vector2f>& pos,
                             const std::vector<float>&) {
          std::vector<sf::Vector2f> acc(pos.size());
          for (size_t i = 0; i < pos.size(); ++i)
              acc[i] = {-pos[i].x, -pos[i].y};
          return acc;
      };
      float dt = 0.01f;
      int steps = static_cast<int>(3.14159f / 2.f / dt);

      // Euler run
      std::vector<Body> euler_bodies = {{1, 1.f, {1.f, 0.f}, {0.f, 0.f}, 1.f}};
      EulerIntegrator euler;
      for (int i = 0; i < steps; ++i) euler.step(euler_bodies, dt, spring_accel);
      float euler_err = std::abs(euler_bodies[0].position.x);

      // RK4 run
      std::vector<Body> rk4_bodies = {{1, 1.f, {1.f, 0.f}, {0.f, 0.f}, 1.f}};
      RK4Integrator rk4;
      for (int i = 0; i < steps; ++i) rk4.step(rk4_bodies, dt, spring_accel);
      float rk4_err = std::abs(rk4_bodies[0].position.x);

      // RK4 error should be orders of magnitude smaller
      assert(rk4_err < euler_err);
  }
  ```

- [ ] **Step 2: Run tests to verify new tests fail**

  Expected: linker error for `RK4Integrator`.

- [ ] **Step 3: Implement `RK4Integrator::step()`**

  Follow the staged RK4 algorithm from the Mathematical Specifications exactly. Key implementation detail: allocate temporary vectors for stage-shifted positions (`std::vector<sf::Vector2f>`, size N). Each stage:
  1. Build shifted positions from original positions + `(h * fraction) * k_x` for that stage.
  2. Call `accel_fn(shifted_positions, masses)` to get stage accelerations.
  3. Compute `k_x` (velocity-derived) and `k_v` (acceleration-derived) for each body.

  After all four stages, apply the weighted update to each body's position and velocity.

- [ ] **Step 4: Run all integrator tests**

  ```bash
  cmake --build build
  ./build/tests/test_integrators
  ```
  Expected: all tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add include/rk4_integrator.hpp src/rk4_integrator.cpp tests/test_integrators.cpp CMakeLists.txt
  git commit -m "feat: RK4Integrator with convergence test"
  ```

---

## Task 5: PhysicsWorld — Gravity, Integration, and Collisions

**Files:**
- Create: `include/physics_world.hpp`
- Create: `src/physics_world.cpp`
- Create: `tests/test_physics.cpp`
- Modify: `CMakeLists.txt` — add `src/physics_world.cpp`
- Modify: `tests/CMakeLists.txt` — add `test_physics`

**Interfaces:**
- Consumes: `Body` (Task 2), `IIntegrator` / `EulerIntegrator` / `RK4Integrator` (Tasks 3–4), `SimConfig` (Task 1)
- Produces:
  ```cpp
  class PhysicsWorld {
  public:
      explicit PhysicsWorld(SimConfig config);
      void set_bodies(std::vector<Body> bodies);
      void step();  // one fixed-timestep physics step
      const std::vector<Body>& bodies() const;
      std::size_t body_count() const;
  private:
      SimConfig config_;
      std::vector<Body> bodies_;
      std::unique_ptr<IIntegrator> integrator_;
      std::vector<sf::Vector2f> compute_accelerations(
          const std::vector<sf::Vector2f>& positions,
          const std::vector<float>& masses) const;
      void resolve_collisions();
  };
  ```

- [ ] **Step 1: Write failing tests in `tests/test_physics.cpp`**

  ```cpp
  void test_two_body_attraction() {
      // Two equal-mass bodies separated along x-axis, both initially at rest.
      // After one step, they should accelerate toward each other.
      SimConfig cfg;
      cfg.G = 1.0f; cfg.epsilon = 0.1f; cfg.dt = 0.01f; cfg.integrator = "euler";
      PhysicsWorld world(cfg);
      world.set_bodies({
          {1, 100.f, {0.f, 0.f}, {0.f, 0.f}, 1.f},
          {2, 100.f, {100.f, 0.f}, {0.f, 0.f}, 1.f}
      });
      world.step();
      auto& b = world.bodies();
      // Body 0 should have moved rightward (toward body 1)
      assert(b[0].position.x > 0.f);
      // Body 1 should have moved leftward (toward body 0)
      assert(b[1].position.x < 100.f);
  }

  void test_collision_merge_reduces_count() {
      SimConfig cfg;
      cfg.G = 1.0f; cfg.epsilon = 0.1f; cfg.dt = 0.01f; cfg.integrator = "euler";
      PhysicsWorld world(cfg);
      // Two bodies overlapping → should merge
      world.set_bodies({
          {1, 10.f, {0.f, 0.f}, {1.f, 0.f}, 5.f},
          {2, 10.f, {3.f, 0.f}, {-1.f, 0.f}, 5.f}  // distance 3 < 5+5
      });
      world.step();
      assert(world.body_count() == 1);
      assert(world.bodies()[0].mass == 20.f);
  }

  void test_single_body_no_crash() {
      SimConfig cfg;
      cfg.G = 1.0f; cfg.epsilon = 0.1f; cfg.dt = 0.01f; cfg.integrator = "rk4";
      PhysicsWorld world(cfg);
      world.set_bodies({{1, 100.f, {50.f, 50.f}, {1.f, 0.f}, 5.f}});
      world.step();  // must not crash
      assert(world.body_count() == 1);
      // Body drifts at constant velocity (no forces)
      assert(std::abs(world.bodies()[0].position.x - 50.01f) < 0.1f);
  }
  ```

- [ ] **Step 2: Run tests to verify they fail**

  Expected: linker error for `PhysicsWorld`.

- [ ] **Step 3: Implement `PhysicsWorld`**

  In the constructor: based on `config_.integrator`, create either `std::make_unique<EulerIntegrator>()` or `std::make_unique<RK4Integrator>()`.

  `compute_accelerations()`: O(N²) nested loop over all pairs. For each body $i$, accumulate acceleration from all bodies $j \neq i$ using the softened gravity formula. Return `std::vector<sf::Vector2f>`.

  `step()`:
  1. Create a lambda wrapping `compute_accelerations` that captures `this` and the config.
  2. Call `integrator_->step(bodies_, config_.dt, accel_lambda)`.
  3. Call `resolve_collisions()`.

  `resolve_collisions()`:
  1. `bool merged = true;`
  2. While `merged`: set `merged = false`, iterate all pairs `(i, j)` where `i < j`. If `check_collision(bodies_[i], bodies_[j])`: replace `bodies_[i]` with `merge(bodies_[i], bodies_[j])`, erase `bodies_[j]`, set `merged = true`, break inner loops (restart scan).

  Edge case: if `bodies_.size() <= 1`, `compute_accelerations` returns a vector of zero-vectors.

- [ ] **Step 4: Run tests**

  ```bash
  cmake --build build
  ./build/tests/test_physics
  ```
  Expected: all tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add include/physics_world.hpp src/physics_world.cpp tests/test_physics.cpp CMakeLists.txt tests/CMakeLists.txt
  git commit -m "feat: PhysicsWorld with gravity, integration, and collision merging"
  ```

---

## Task 6: Simulation Presets

**Files:**
- Create: `include/sim_presets.hpp`
- Create: `src/sim_presets.cpp`
- Create: `tests/test_presets.cpp`
- Modify: `CMakeLists.txt` — add `src/sim_presets.cpp`
- Modify: `tests/CMakeLists.txt` — add `test_presets`

**Interfaces:**
- Consumes: `Body` (Task 2), `SimConfig` (Task 1)
- Produces:
  ```cpp
  namespace presets {
      std::vector<Body> create_stable_orbit(const SimConfig& cfg);
      std::vector<Body> create_solar_system(const SimConfig& cfg);
      std::vector<Body> create_random_cloud(const SimConfig& cfg, unsigned int seed = 42);
  }
  ```

- [ ] **Step 1: Write failing tests in `tests/test_presets.cpp`**

  ```cpp
  void test_stable_orbit_has_two_bodies() {
      SimConfig cfg;
      auto bodies = presets::create_stable_orbit(cfg);
      assert(bodies.size() == 2);
      // Central body should be more massive
      assert(bodies[0].mass > bodies[1].mass);
  }
  void test_solar_system_body_count() {
      SimConfig cfg;
      auto bodies = presets::create_solar_system(cfg);
      assert(bodies.size() >= 4);  // sun + at least 3 planets
  }
  void test_random_cloud_deterministic() {
      SimConfig cfg;
      auto a = presets::create_random_cloud(cfg, 42);
      auto b = presets::create_random_cloud(cfg, 42);
      assert(a.size() == b.size());
      for (size_t i = 0; i < a.size(); ++i)
          assert(a[i].position.x == b[i].position.x);
  }
  ```

- [ ] **Step 2: Run tests to verify they fail**

- [ ] **Step 3: Implement presets**

  `create_stable_orbit`: Place a heavy central body at the center of the physics space (e.g., position `{400, 360}`). Compute the circular orbit velocity for a lighter orbiting body at distance $d$: $v = \sqrt{G \cdot M / d}$. Place the orbiter at `{400 + d, 360}` with velocity `{0, v}` (perpendicular to the radial direction). Use `cfg.G` for the gravitational constant.

  `create_solar_system`: Central body (large mass, position center), 4–5 orbiting bodies at increasing distances with circular orbit velocities computed as above. Vary masses and radii.

  `create_random_cloud`: Use `std::mt19937` seeded with the `seed` parameter. Generate 50–100 bodies with positions uniformly distributed in a region, small random velocities, masses in a range, and radii proportional to `cbrt(mass)` (or similar).

- [ ] **Step 4: Run tests**

  ```bash
  cmake --build build
  ./build/tests/test_presets
  ```
  Expected: all tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add include/sim_presets.hpp src/sim_presets.cpp tests/test_presets.cpp CMakeLists.txt tests/CMakeLists.txt
  git commit -m "feat: simulation presets (orbit, solar, random cloud)"
  ```

---

## Task 7: SimulationApp — SFML 3 Window, Event Loop, Rendering, and HUD

**Files:**
- Create: `include/simulation_app.hpp`
- Create: `src/simulation_app.cpp`
- Modify: `src/main.cpp` — replace placeholder with full app launch
- Modify: `CMakeLists.txt` — add `src/simulation_app.cpp` and `src/sim_presets.cpp` to sources

**Interfaces:**
- Consumes: `PhysicsWorld` (Task 5), `SimConfig` (Task 1), presets (Task 6), `Body` (Task 2)
- Produces:
  ```cpp
  class SimulationApp {
  public:
      explicit SimulationApp(SimConfig config);
      void run();  // enters main loop, returns on window close
  private:
      SimConfig config_;
      PhysicsWorld world_;
      sf::RenderWindow window_;
      sf::Font font_;
      bool paused_ = false;
      unsigned int ups_count_ = 0;  // updates-per-second counter
      unsigned int fps_count_ = 0;  // frames-per-second counter
      unsigned int ups_display_ = 0;
      unsigned int fps_display_ = 0;

      void handle_events();
      void update(float dt);
      void render();
      sf::Vector2f world_to_screen(sf::Vector2f world_pos) const;
  };
  ```

- [ ] **Step 1: Implement `SimulationApp` constructor**

  - Construct `window_` as `sf::RenderWindow(sf::VideoMode({config_.window_width, config_.window_height}), "N-Body Simulator")`.
  - Load `font_` from a bundled font file. Strategy: embed a path to a system font (e.g., `/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf`), or use SFML 3's `sf::Font` constructor with a path string. Wrap in a check; if the font fails to load, print a warning and skip HUD text rendering.
  - Construct `world_` with `config_`.
  - Select and set bodies via the preset matching `config_.preset`, calling the appropriate `presets::` function.

- [ ] **Step 2: Implement `handle_events()`**

  SFML 3 event polling pattern:
  ```cpp
  while (const auto event = window_.pollEvent()) {
      if (event->is<sf::Event::Closed>())
          window_.close();
      if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
          if (key->scancode == sf::Keyboard::Scancode::Escape)
              window_.close();
          if (key->scancode == sf::Keyboard::Scancode::Space)
              paused_ = !paused_;
      }
  }
  ```

- [ ] **Step 3: Implement `world_to_screen()`**

  The physics coordinate space is the same as the window pixel space (1:1 mapping). Bodies with positions outside the window simply render off-screen. This is the simplest correct approach. If a view transform is desired later, use `sf::View`, but for now: `return world_pos;` directly.

- [ ] **Step 4: Implement `render()`**

  ```
  window_.clear(sf::Color::Black);
  for (const auto& body : world_.bodies()) {
      sf::CircleShape shape(body.radius);
      shape.setFillColor(color_for_body(body));  // pick color by mass or id
      shape.setOrigin({body.radius, body.radius});  // center the circle
      shape.setPosition(world_to_screen(body.position));
      window_.draw(shape);
  }
  // HUD
  draw_hud();
  window_.display();
  ```

  `draw_hud()`: Create `sf::Text` objects (SFML 3: font as first arg). Display FPS, UPS, body count, paused state. Position at top-left corner. Guard: if font failed to load, skip.

  `color_for_body()`: Simple mass-based color. E.g., if `mass > 1000` → yellow (star), else white. A helper free function or lambda.

- [ ] **Step 5: Implement `run()` with fixed-timestep accumulator**

  ```
  sf::Clock clock;
  sf::Time accumulator = sf::Time::Zero;
  sf::Time physics_dt = sf::seconds(config_.dt);
  sf::Time fps_timer = sf::Time::Zero;

  while (window_.isOpen()) {
      sf::Time frame_time = clock.restart();
      // Cap frame_time to avoid spiral of death
      if (frame_time > sf::seconds(0.1f))
          frame_time = sf::seconds(0.1f);

      fps_timer += frame_time;
      if (fps_timer >= sf::seconds(1.0f)) {
          fps_display_ = fps_count_;
          ups_display_ = ups_count_;
          fps_count_ = 0;
          ups_count_ = 0;
          fps_timer -= sf::seconds(1.0f);
      }

      handle_events();

      if (!paused_) {
          accumulator += frame_time;
          while (accumulator >= physics_dt) {
              world_.step();
              accumulator -= physics_dt;
              ++ups_count_;
          }
      }

      render();
      ++fps_count_;
  }
  ```

- [ ] **Step 6: Update `src/main.cpp`**

  Parse command-line arguments for integrator and preset selection:
  ```
  Usage: nbody-sim [--integrator euler|rk4] [--preset orbit|solar|random]
  ```
  Default: `euler`, `orbit`. Construct `SimConfig` from args, create `SimulationApp`, call `run()`.

- [ ] **Step 7: Update `CMakeLists.txt` sources**

  The `nbody-sim` target sources should be:
  ```cmake
  add_executable(nbody-sim
      src/main.cpp
      src/body.cpp
      src/euler_integrator.cpp
      src/rk4_integrator.cpp
      src/physics_world.cpp
      src/sim_presets.cpp
      src/simulation_app.cpp
  )
  ```

- [ ] **Step 8: Build, run, and visually verify**

  ```bash
  cmake --build build
  ./build/nbody-sim --integrator rk4 --preset orbit
  ```
  Expected: Window opens. Two bodies visible. One orbits the other. Space pauses/resumes. Escape closes window. HUD shows FPS, UPS, body count. No crash.

  ```bash
  ./build/nbody-sim --integrator euler --preset random
  ```
  Expected: Random cloud of bodies. Bodies attract each other, merge on collision. Body count decreases over time.

- [ ] **Step 9: Commit**

  ```bash
  git add include/simulation_app.hpp src/simulation_app.cpp src/main.cpp CMakeLists.txt
  git commit -m "feat: SimulationApp with SFML 3 rendering, fixed-timestep loop, HUD, and CLI args"
  ```

---

## Task 8: Integration Testing and Polish

**Files:**
- Modify: `tests/test_physics.cpp` — add orbit stability test
- Modify: `src/simulation_app.cpp` — any polish fixes

**Interfaces:**
- Consumes: all prior tasks
- Produces: passing integration tests and a stable, polished simulation

- [ ] **Step 1: Add two-body orbit stability test**

  In `tests/test_physics.cpp`:
  ```cpp
  void test_circular_orbit_stability_rk4() {
      SimConfig cfg;
      cfg.G = 1.0f; cfg.epsilon = 0.1f; cfg.dt = 0.001f; cfg.integrator = "rk4";
      PhysicsWorld world(cfg);
      float M = 10000.f, m = 1.f, d = 200.f;
      float v_orbit = std::sqrt(cfg.G * M / d);
      world.set_bodies({
          {1, M, {400.f, 360.f}, {0.f, 0.f}, 10.f},
          {2, m, {400.f + d, 360.f}, {0.f, v_orbit}, 2.f}
      });
      float initial_dist = d;
      // Run for many steps (~1 orbit period)
      int steps = static_cast<int>(2.0f * 3.14159f * d / v_orbit / cfg.dt);
      for (int i = 0; i < steps; ++i)
          world.step();
      // After one full orbit, distance from center should be close to initial
      auto& b = world.bodies();
      float dx = b[1].position.x - b[0].position.x;
      float dy = b[1].position.y - b[0].position.y;
      float final_dist = std::sqrt(dx*dx + dy*dy);
      // Allow 5% deviation for RK4 with this dt
      assert(std::abs(final_dist - initial_dist) / initial_dist < 0.05f);
  }
  ```

- [ ] **Step 2: Run the orbit stability test**

  ```bash
  cmake --build build
  ./build/tests/test_physics
  ```
  Expected: passes. If it fails, reduce `cfg.dt` or increase epsilon slightly and document the change.

- [ ] **Step 3: Run the full application with each preset and integrator combination**

  ```bash
  ./build/nbody-sim --integrator euler --preset orbit
  ./build/nbody-sim --integrator rk4 --preset orbit
  ./build/nbody-sim --integrator rk4 --preset solar
  ./build/nbody-sim --integrator euler --preset random
  ./build/nbody-sim --integrator rk4 --preset random
  ```
  Verify for each: window opens, bodies render, physics runs, pause works, HUD displays, window closes cleanly.

- [ ] **Step 4: Edge case — run with zero bodies**

  Add a temporary test or verify in-app: if a preset returns zero bodies, the simulation should run without crashing (empty render, HUD shows body count 0).

- [ ] **Step 5: Commit**

  ```bash
  git add tests/test_physics.cpp
  git commit -m "test: orbit stability integration test and edge case verification"
  ```

- [ ] **Step 6: Final commit and tag**

  ```bash
  git add -A
  git status  # verify clean
  git tag v1.0.0
  ```
