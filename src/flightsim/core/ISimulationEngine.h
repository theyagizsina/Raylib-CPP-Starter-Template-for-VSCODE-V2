#pragma once

#include "EngineState.h"
#include "FlightControls.h"

class ISimulationEngine {
public:
    virtual ~ISimulationEngine() = default;

    virtual void initializeBodyState() = 0;
    virtual void step(double deltaTime, const FlightControls& controls) = 0;
    virtual void resetTime(double timeSeconds = 0.0) = 0;
    virtual double getSimulationTime() const = 0;
};
