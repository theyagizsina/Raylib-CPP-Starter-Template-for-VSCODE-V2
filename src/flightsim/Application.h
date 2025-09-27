#pragma once

#include "core/EngineState.h"
#include "core/SimulationConfig.h"
#include "physics/FlightDynamics.h"
#include "physics/Quaternion.h"
#include "physics/Atmosphere.h"
#include "ofm/OFMInterface.h"
#include "io/DataLogger.h"
#include "io/ConsoleOutput.h"

class Application {
private:
    EngineState engineState;
    OFMInterface ofmInterface;
    DataLogger dataLogger;
    State6 bodyState;

    void initializeBodyState();
    void simulationStep(double simTime);
    void calculateAeroForces(double& FxB, double& FyB, double& FzB,
                           double& MxB, double& MyB, double& MzB);
    void addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB);
    void updatePhysics(double subDt, double FxB, double FyB, double FzB,
                      double MxB, double MyB, double MzB);
    void updatePosition(double subDt);

public:
    Application();
    ~Application();

    bool initialize();
    int run();
};