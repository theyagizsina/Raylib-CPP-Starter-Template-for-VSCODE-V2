#pragma once

#include "core/EngineState.h"
#include "core/SimulationConfig.h"
#include "ofm/OFMInterface.h"
#include "io/DataLogger.h"
#include "io/ConsoleOutput.h"
#include "core/SimulationEngine.h"
#include "core/FlightControls.h"

class Application {
private:
    EngineState engineState;
    OFMInterface ofmInterface;
    DataLogger dataLogger;
    State6 bodyState;
    SimulationEngine simulationEngine;
    FlightControls controls;

public:
    Application();
    ~Application();

    bool initialize();
    int run();
};