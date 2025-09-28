#include "FlightSimulationSubsystem.h"

#include <utility>

#include "../../engine/EventBus.h"
#include "SimulationEvents.h"

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

void FlightSimulationSubsystem::onFixedUpdate(engine::EngineContext& context, double fixedDeltaTime)
{
    simulationEngine.step(fixedDeltaTime, controls);

    flightsim::SimulationFixedStepEvent evt{};
    evt.fixedDeltaTime = fixedDeltaTime;
    evt.simulationTime = simulationEngine.getSimulationTime();
    evt.stateSnapshot = engineState;
    context.eventBus.publish(std::move(evt));
}
