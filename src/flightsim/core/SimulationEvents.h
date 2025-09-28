#pragma once

#include "EngineState.h"

namespace flightsim {

struct SimulationFixedStepEvent {
    double fixedDeltaTime;
    double simulationTime;
    EngineState stateSnapshot;
};

} // namespace flightsim
