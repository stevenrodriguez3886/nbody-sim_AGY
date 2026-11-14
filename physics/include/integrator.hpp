#pragma once

#include "body.hpp"
#include <functional>
#include <vector>
#include <SFML/System/Vector2.hpp>

// Callback type: given a vector of positions and masses, return vector of accelerations
using AccelerationFn = std::function<std::vector<sf::Vector2f>(
    const std::vector<sf::Vector2f>& positions,
    const std::vector<float>& masses)>;

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual void step(std::vector<Body>& bodies, float dt,
                      const AccelerationFn& accel_fn) = 0;
};
