#pragma once
#include <raylib.h>
#include "flightsim/core/EngineState.h"
#include "flightsim/physics/FlightDynamics.h"
#include "flightsim/physics/Quaternion.h"
#include "flightsim/physics/Atmosphere.h"
#include "flightsim/ofm/OFMInterface.h"
#include "flightsim/io/DataLogger.h"
#include <vector>

// Use typedef to avoid conflict with Raylib's CameraMode
typedef enum {
    FLIGHT_CAMERA_EXTERNAL = 0,
    FLIGHT_CAMERA_COCKPIT
} FlightCameraType;

class FlightSimulator {
private:
    EngineState engineState;
    OFMInterface ofmInterface;
    DataLogger dataLogger;
    State6 bodyState;

    // Visualization
    Camera3D camera;
    std::vector<Vector3> flightPath;
    bool isSimulationRunning;
    double simulationTime;
    
    // Camera system
    FlightCameraType cameraMode;
    Vector2 mouseLastPos;
    float cameraDistance;
    float cameraYaw;
    float cameraPitch;
    
    // Flight data for display
    double currentAoA;
    double currentBeta;
    double currentAirspeed;
    double currentClimbRate;
    double lastAltitude;
    
    // Control system
    struct FlightControls {
        float aileron;      // -1.0 to 1.0 (left/right roll)
        float elevator;     // -1.0 to 1.0 (nose up/down)
        float rudder;       // -1.0 to 1.0 (nose left/right)
        float throttle;     // 0.0 to 1.0 (engine power)
    } controls;

    // Default configuration for easier testing
    void setDefaultConfiguration();
    void initializeBodyState();
    void simulationStep(double deltaTime);
    void calculateAeroForces(double& FxB, double& FyB, double& FzB,
                           double& MxB, double& MyB, double& MzB);
    void addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB);
    void updatePhysics(double subDt, double FxB, double FyB, double FzB,
                      double MxB, double MyB, double MzB);
    void updatePosition(double subDt);

    // Visualization methods
    void drawAircraft();
    void drawFlightPath();
    void drawEnvironment();
    void drawInstruments();
    void drawHUD();
    void drawFlightPathMarker();
    void drawAttitudeIndicator();
    void updateCameraSystem();
    void updateFlightData();
    void updateControls();
    void sendControlsToOFM();

public:
    FlightSimulator();
    ~FlightSimulator();

    bool initialize();
    void update();
    void draw();
    void handleInput();
};