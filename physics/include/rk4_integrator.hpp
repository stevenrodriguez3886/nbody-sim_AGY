#pragma once

#include "integrator.hpp"

class RK4Integrator : public IIntegrator {
public:
    void step(std::vector<Body>& bodies, float dt,
              const AccelerationFn& accel_fn) override;
};
