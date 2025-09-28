#pragma once

#include "../../engine/Subsystem.h"

#include "SimulationEngine.h"

class FlightSimulationSubsystem : public engine::ISubsystem {
public:
    FlightSimulationSubsystem(SimulationEngine& simulationEngine,
                              EngineState& engineState,
                              FlightControls& controls);

    const char* getName() const override { return "FlightSimulation"; }

    bool onInitialize(engine::EngineContext& context) override;
    void onFixedUpdate(engine::EngineContext& context, double fixedDeltaTime) override;

private:
    SimulationEngine& simulationEngine;
    EngineState& engineState;
    FlightControls& controls;
};
