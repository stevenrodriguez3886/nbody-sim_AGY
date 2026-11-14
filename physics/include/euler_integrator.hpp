#pragma once

#include "integrator.hpp"

class EulerIntegrator : public IIntegrator {
public:
    void step(std::vector<Body>& bodies, float dt,
              const AccelerationFn& accel_fn) override;
};
