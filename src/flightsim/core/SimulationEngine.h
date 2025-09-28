#pragma once

#include <functional>
#include <string>

#include "ISimulationEngine.h"
#include "../physics/FlightDynamics.h"
#include "../physics/Quaternion.h"
#include "../physics/Atmosphere.h"
#include "../io/DataLogger.h"
#include "../ofm/OFMInterface.h"

class SimulationEngine : public ISimulationEngine {
public:
    enum class ForceKind {
        Aerodynamic,
        Gravity
    };

    enum class MomentKind {
        Induced,
        Direct
    };

    struct ForceVectorSample {
        double px;
        double py;
        double pz;
        double fx;
        double fy;
        double fz;
        ForceKind kind;
    };

    struct MomentVectorSample {
        double px;
        double py;
        double pz;
        double mx;
        double my;
        double mz;
        MomentKind kind;
    };

    using ForceCallback = std::function<void(const ForceVectorSample&)>;
    using MomentCallback = std::function<void(const MomentVectorSample&)>;

    SimulationEngine(EngineState& engineState,
                     State6& bodyState,
                     OFMInterface& ofmInterface,
                     DataLogger* dataLogger = nullptr);

    void setForceCallback(ForceCallback forceCb, MomentCallback momentCb);
    void setDataLogger(DataLogger* logger);

    void initializeBodyState() override;
    void step(double deltaTime, const FlightControls& controls) override;
    void resetTime(double timeSeconds = 0.0) override;
    double getSimulationTime() const override;

private:
    void sendControlsToOFM(const FlightControls& controls);
    void calculateAeroForces(double& FxB, double& FyB, double& FzB,
                             double& MxB, double& MyB, double& MzB);
    void addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB);
    void updatePhysics(double subDt, double FxB, double FyB, double FzB,
                       double MxB, double MyB, double MzB);
    void updatePosition(double subDt);
    void logFlightData();

    EngineState& engineState;
    State6& bodyState;
    OFMInterface& ofmInterface;
    DataLogger* dataLogger;

    double simulationTime;
    int logCounter;

    ForceCallback forceCallback;
    MomentCallback momentCallback;
};
