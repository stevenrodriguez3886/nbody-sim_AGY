#pragma once

#include <cstdint>
#include <string>

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
