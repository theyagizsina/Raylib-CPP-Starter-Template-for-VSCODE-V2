#include "FlightSimulationSubsystem.h"

FlightSimulationSubsystem::FlightSimulationSubsystem(SimulationEngine& simulationEngineRef,
                                                     EngineState& engineStateRef,
                                                     FlightControls& controlsRef)
    : simulationEngine(simulationEngineRef),
      engineState(engineStateRef),
      controls(controlsRef)
{
}

bool FlightSimulationSubsystem::onInitialize(engine::EngineContext& /*context*/)
{
    simulationEngine.initializeBodyState();
    simulationEngine.resetTime();
    return true;
}

void FlightSimulationSubsystem::onFixedUpdate(engine::EngineContext& /*context*/, double fixedDeltaTime)
{
    simulationEngine.step(fixedDeltaTime, controls);
}
