#pragma once

#include <vector>
#include "body.hpp"
#include "sim_config.hpp"

namespace presets {

std::vector<Body> create_stable_orbit(const SimConfig& cfg);
std::vector<Body> create_solar_system(const SimConfig& cfg);
std::vector<Body> create_random_cloud(const SimConfig& cfg, unsigned int seed = 42);

} // namespace presets
