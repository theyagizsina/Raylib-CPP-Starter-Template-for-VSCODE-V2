#pragma once
#include <raylib.h>
#include <cmath>
#include "flightsim/core/EngineState.h"
#include "flightsim/physics/FlightDynamics.h"
#include "flightsim/physics/Quaternion.h"
#include "flightsim/physics/Atmosphere.h"
#include "flightsim/ofm/OFMInterface.h"
#include "flightsim/io/DataLogger.h"
#include <vector>
#include <string>
#include <array>

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
    
    // 3D Model
    Model aircraftModel;
    Matrix aircraftModelBaseTransform;
    float customModelScale;
    bool hasCustomModel;
    
    // Camera system
    FlightCameraType cameraMode;
    Vector2 mouseLastPos;
    float cameraDistance;
    float cameraYaw;
    float cameraPitch;
    
    // Head tracking system for cockpit view
    struct HeadLook {
        float yaw = 0.0f;          // Head yaw: -90° to +90°  
        float pitch = 0.0f;        // Head pitch: -85° to +85°
        float sensitivity = 0.3f;  // Mouse sensitivity
        float smoothing = 0.85f;   // Damping factor (0 = no smoothing, 0.99 = heavy smoothing)
        
        // Cockpit position relative to aircraft center (body coordinates)
        Vector3 headOffset = {3.80267f, 0.0f, -0.631895f}; // X=forward, Y=right, Z=down
        
        // Input smoothing buffers
        float targetYaw = 0.0f;
        float targetPitch = 0.0f;
    } headLook;
    
    bool mouseHeadLookEnabled = false;
    
    // FOV calculation system
    struct FOVConfig {
        float screenWidthMM = 598.0f;    // 27" monitor width in mm
        float screenHeightMM = 336.0f;   // 27" monitor height in mm  
        float eyeDistanceMM = 600.0f;    // Eye distance from screen in mm
        float baseFOV = 45.0f;           // Fallback FOV if auto calculation disabled
        bool autoFOV = true;             // Enable automatic FOV calculation
        
        // Cockpit FOV zoom control
        float cockpitFOV = 45.0f;        // Current cockpit FOV
        float minFOV = 10.0f;            // Minimum zoom (telescopic)
        float maxFOV = 120.0f;           // Maximum wide angle
        float zoomSpeed = 5.0f;          // FOV change per scroll step
        
        float calculateVerticalFOV() const {
            if (!autoFOV) return baseFOV;
            // vFOV = 2 * atan(screenHeight / (2 * eyeDistance))
            float vFOVrad = 2.0f * (float)atan(screenHeightMM / (2.0f * eyeDistanceMM));
            return vFOVrad * 180.0f / 3.14159265358979323846f; // Convert to degrees
        }
        
        float getCockpitFOV() const {
            return cockpitFOV;
        }
        
        void adjustCockpitFOV(float scrollDelta) {
            cockpitFOV -= scrollDelta * zoomSpeed; // Negative for zoom in on scroll up
            if (cockpitFOV < minFOV) cockpitFOV = minFOV;
            if (cockpitFOV > maxFOV) cockpitFOV = maxFOV;
        }
        
        void resetCockpitFOV() {
            cockpitFOV = calculateVerticalFOV();
        }
    } fovConfig;

    // Gamepad state
    int activeGamepadIndex;
    std::string activeGamepadName;
    int lastGamepadAxisCount;
    std::array<float, 8> lastGamepadAxes;
    float lastGamepadRoll;
    float lastGamepadPitch;
    
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
    
    // Force vector visualization system
    struct ForceVector {
        Vector3 position;      // Application point (body frame)
        Vector3 force;         // Force vector (body frame)
        Color color;           // Color for visualization
        std::string label;     // Force type label
        bool isLocal;          // True for local forces, false for moments
        
        ForceVector(Vector3 pos, Vector3 f, Color c, const std::string& l, bool local = true) 
            : position(pos), force(f), color(c), label(l), isLocal(local) {}
    };
    
    struct ForceVisualization {
        std::vector<ForceVector> forces;
        std::vector<ForceVector> moments;
        bool showForces = false;
        bool showMoments = false;
        bool showLabels = true;
        float forceScale = 0.01f;      // Scale factor for force vectors
        float momentScale = 0.05f;     // Scale factor for moment vectors
        float vectorThickness = 3.0f;  // Line thickness
        
        void clear() { 
            forces.clear(); 
            moments.clear(); 
        }
        
        void addForce(const Vector3& pos, const Vector3& force, Color color, const std::string& label) {
            forces.emplace_back(pos, force, color, label, true);
        }
        
        void addMoment(const Vector3& pos, const Vector3& moment, Color color, const std::string& label) {
            moments.emplace_back(pos, moment, color, label, false);
        }
    } forceViz;

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
    void drawForceVectors();
    bool loadAircraftModel(const char* modelPath);
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