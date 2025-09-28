#include "FlightSimulator.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <memory>
#include <raymath.h>

#include "flightsim/core/FlightSimulationSubsystem.h"
#include "engine/EngineConfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

FlightSimulator::FlightSimulator()
    : isSimulationRunning(false),
      simulationEngine(engineState, bodyState, ofmInterface, &dataLogger),
      aircraftModelBaseTransform(MatrixIdentity()), customModelScale(1.0f), hasCustomModel(false),
    cameraMode(FLIGHT_CAMERA_EXTERNAL), cameraDistance(150.0f), cameraYaw(0.0f), cameraPitch(-20.0f),
    activeGamepadIndex(-1), lastGamepadAxisCount(0), lastGamepadRoll(0.0f), lastGamepadPitch(0.0f),
    currentAoA(0.0), currentBeta(0.0), currentAirspeed(0.0), currentClimbRate(0.0), lastAltitude(0.0)
{
    mouseLastPos = { 0, 0 };
    lastGamepadAxes.fill(0.0f);
    
    // Initialize flight controls
    controls.aileron = 0.0f;
    controls.elevator = 0.0f;
    controls.rudder = 0.0f;
    controls.throttle = 0.5f;  // Start with 50% throttle

    simulationEngine.setForceCallback(
        [this](const SimulationEngine::ForceVectorSample& sample) { handleForceSample(sample); },
        [this](const SimulationEngine::MomentVectorSample& sample) { handleMomentSample(sample); }
    );
}

FlightSimulator::~FlightSimulator()
{
    if (simulationStepHandle.valid()) {
        coreEngine.getEventBus().unsubscribe(simulationStepHandle);
    }

    coreEngine.shutdown();

    if (hasCustomModel) {
        UnloadModel(aircraftModel);
    }
}

bool FlightSimulator::initialize()
{
    // Set default configuration instead of asking for user input
    setDefaultConfiguration();

    if (!ofmInterface.initialize()) {
        std::cout << "Failed to initialize OFM interface" << std::endl;
        return false;
    }

    if (!dataLogger.initialize("flight_log.csv")) {
        std::cout << "Failed to initialize data logger" << std::endl;
        return false;
    }

    simulationEngine.setDataLogger(&dataLogger);

    ofmInterface.init();
    ofmInterface.setMassState(
        engineState.mass,
        engineState.cmX, engineState.cmY, engineState.cmZ,
        engineState.moiX, engineState.moiY, engineState.moiZ
    );

    engine::EngineConfig engineConfig;
    engineConfig.fixedTimeStep = engineState.dt;
    engineConfig.maxFixedStepsPerFrame = 4;
    engineConfig.maxDeltaTime = 0.1;
    coreEngine.setConfig(engineConfig);

    auto flightSubsystem = std::make_unique<FlightSimulationSubsystem>(simulationEngine, engineState, controls);
    simulationSubsystem = flightSubsystem.get();
    coreEngine.registerSubsystem(std::move(flightSubsystem));

    if (!coreEngine.initialize()) {
        std::cout << "Failed to initialize core engine" << std::endl;
        return false;
    }

    coreEngine.start();

    simulationStepHandle = coreEngine.getEventBus().subscribe<flightsim::SimulationFixedStepEvent>(
        [this](const flightsim::SimulationFixedStepEvent& evt) { onSimulationFixedStep(evt); });
    lastFixedStepDelta = engineState.dt;
    lastSimulationTimeEvent = 0.0;
    hasSimulationEventData = false;

    // Setup 3D camera with automatic FOV calculation
    camera.position = (Vector3){ 100.0f, 50.0f, 100.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = fovConfig.calculateVerticalFOV(); // Use calculated FOV
    camera.projection = CAMERA_PERSPECTIVE;
    
    // Initialize cockpit FOV to calculated value
    fovConfig.cockpitFOV = fovConfig.calculateVerticalFOV();
    
    // Initialize camera system
    cameraMode = FLIGHT_CAMERA_EXTERNAL;
    cameraDistance = 150.0f;
    cameraYaw = 0.0f;
    cameraPitch = -20.0f;
    
    // Initialize flight data
    lastAltitude = -engineState.Z;
    activeGamepadIndex = -1;
    activeGamepadName.clear();
    lastGamepadRoll = 0.0f;
    lastGamepadPitch = 0.0f;
    lastGamepadAxisCount = 0;
    lastGamepadAxes.fill(0.0f);

    isSimulationRunning = true;

    return true;
}

bool FlightSimulator::loadAircraftModel(const char* modelPath)
{
    if (hasCustomModel) {
        UnloadModel(aircraftModel);
        hasCustomModel = false;
    }
    
    aircraftModel = LoadModel(modelPath);
    
    if (aircraftModel.meshCount > 0) {
        aircraftModelBaseTransform = aircraftModel.transform;
        customModelScale = 1.0f;
        hasCustomModel = true;
        std::cout << "Aircraft model loaded successfully: " << modelPath << std::endl;
        std::cout << "Meshes: " << aircraftModel.meshCount << ", Materials: " << aircraftModel.materialCount << std::endl;
        return true;
    } else {
        std::cout << "Failed to load aircraft model: " << modelPath << std::endl;
        return false;
    }
}

void FlightSimulator::setDefaultConfiguration()
{
    // Default configuration for easy testing
    engineState.X = 0; engineState.Y = 0; engineState.Z = -10000; // Start at 1000m altitude
    engineState.Vx = 80; engineState.Vy = 0; engineState.Vz = 0; // 50 m/s forward

    // Level flight attitude (no initial roll angle)
    engineState.q.w = 1; engineState.q.x = 0; engineState.q.y = 0; engineState.q.z = 0;

    // Initial angular velocities - roll moment for aileron deflection
    engineState.p = 0; // Roll angular velocity (rad/s) - aileron creates roll moment
    engineState.q_ = 0;  // No pitch moment
    engineState.r = 0;   // No yaw moment
    
    engineState.mass = 3266; // 1000 kg aircraft
    engineState.cmX = 0.213941; engineState.cmY = 0; engineState.cmZ = 0.090697;
    engineState.moiX = 1438; engineState.moiY = 25874; engineState.moiZ = 26779;
    engineState.windX = 0; engineState.windY = 0; engineState.windZ = 0;
    engineState.dt = 0.016; // 60 FPS
    engineState.integratorType = 1; // RK4
    engineState.subSteps = 1;
}

void FlightSimulator::update()
{
    if (!isSimulationRunning) return;

    handleInput();
    updateControls();
    
    double deltaTime = GetFrameTime();
    forceViz.clear();
    coreEngine.tick(deltaTime);
    
    updateCameraSystem();

    // Stop simulation if aircraft hits ground
    if (engineState.Z >= 0.0) {
        isSimulationRunning = false;
        coreEngine.synchronizeClock();
        std::cout << "Ground contact. Simulation stopped." << std::endl;
    }
}

void FlightSimulator::onSimulationFixedStep(const flightsim::SimulationFixedStepEvent& evt)
{
    const double previousAltitude = lastAltitude;
    const double previousSimulationTime = lastSimulationTimeEvent;

    const double altitude = -evt.stateSnapshot.Z;
    double deltaSimTime = evt.simulationTime - previousSimulationTime;
    if (deltaSimTime <= 1e-9) {
        deltaSimTime = evt.fixedDeltaTime;
    }

    if (hasSimulationEventData) {
        if (deltaSimTime > 1e-9) {
            currentClimbRate = (altitude - previousAltitude) / deltaSimTime;
        } else {
            currentClimbRate = 0.0;
        }
    } else {
        currentClimbRate = 0.0;
        hasSimulationEventData = true;
    }

    lastAltitude = altitude;
    lastFixedStepDelta = evt.fixedDeltaTime;
    lastSimulationTimeEvent = evt.simulationTime;

    const double u = bodyState.u;
    const double v = bodyState.v;
    const double w = bodyState.w;
    currentAirspeed = std::sqrt(u * u + v * v + w * w);

    if (currentAirspeed > 1e-6) {
        currentAoA = std::atan2(w, u);
        double betaArg = v / currentAirspeed;
        if (betaArg > 1.0) {
            betaArg = 1.0;
        } else if (betaArg < -1.0) {
            betaArg = -1.0;
        }
        currentBeta = std::asin(betaArg);
    } else {
        currentAoA = 0.0;
        currentBeta = 0.0;
    }

    flightPath.push_back((Vector3){
        (float)evt.stateSnapshot.X,
        (float)(-evt.stateSnapshot.Z),
        (float)evt.stateSnapshot.Y
    });

    if (flightPath.size() > 1000) {
        flightPath.erase(flightPath.begin());
    }
}


void FlightSimulator::handleInput()
{
    if (IsKeyPressed(KEY_SPACE)) {
        isSimulationRunning = !isSimulationRunning;
        if (isSimulationRunning) {
            coreEngine.synchronizeClock();
        }
    }

    if (IsKeyPressed(KEY_R)) {
        // Reset simulation
        setDefaultConfiguration();
        engine::EngineConfig engineConfig = coreEngine.getConfig();
        engineConfig.fixedTimeStep = engineState.dt;
        coreEngine.setConfig(engineConfig);
        simulationEngine.initializeBodyState();
        flightPath.clear();
        simulationEngine.resetTime();
        lastAltitude = -engineState.Z;
        lastSimulationTimeEvent = 0.0;
        lastFixedStepDelta = engineState.dt;
        hasSimulationEventData = false;
        currentAoA = 0.0;
        currentBeta = 0.0;
        currentAirspeed = 0.0;
        currentClimbRate = 0.0;
        isSimulationRunning = true;
        coreEngine.synchronizeClock();
    }
    
    // Load custom model - L key
    if (IsKeyPressed(KEY_L)) {
        // Try to load a model from the models directory
        if (loadAircraftModel("models/aircraft.obj") || 
            loadAircraftModel("models/aircraft.glb") ||
            loadAircraftModel("models/aircraft.gltf") ||
            loadAircraftModel("aircraft.obj") ||
            loadAircraftModel("aircraft.glb") ||
            loadAircraftModel("aircraft.gltf")) {
            std::cout << "Custom aircraft model loaded!" << std::endl;
        } else {
            std::cout << "No aircraft model found. Place your model as 'models/aircraft.obj', 'models/aircraft.glb', or in root directory" << std::endl;
        }
    }
    
    // Toggle between custom and basic model - M key
    if (IsKeyPressed(KEY_M) && hasCustomModel) {
        // This just shows how you could toggle, but we'll keep the custom model active
        std::cout << "Using custom aircraft model" << std::endl;
    }
    
    // Camera mode switching
    if (IsKeyPressed(KEY_C)) {
        cameraMode = (cameraMode == FLIGHT_CAMERA_EXTERNAL) ? FLIGHT_CAMERA_COCKPIT : FLIGHT_CAMERA_EXTERNAL;
    }
    
    // Toggle head look mode in cockpit (H key)
    if (IsKeyPressed(KEY_H) && cameraMode == FLIGHT_CAMERA_COCKPIT) {
        mouseHeadLookEnabled = !mouseHeadLookEnabled;
        if (mouseHeadLookEnabled) {
            SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
        } else {
            SetMouseCursor(MOUSE_CURSOR_DEFAULT);
        }
    }
    
    // Head look controls for cockpit view
    if (cameraMode == FLIGHT_CAMERA_COCKPIT && mouseHeadLookEnabled && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 mousePos = GetMousePosition();
        Vector2 mouseDelta = { mousePos.x - mouseLastPos.x, mousePos.y - mouseLastPos.y };
        
        // Update target head orientation with mouse movement
        headLook.targetYaw += mouseDelta.x * headLook.sensitivity;
        headLook.targetPitch += mouseDelta.y * headLook.sensitivity;
        
        // Clamp head movement limits
        if (headLook.targetYaw > 90.0f) headLook.targetYaw = 90.0f;
        if (headLook.targetYaw < -90.0f) headLook.targetYaw = -90.0f;
        if (headLook.targetPitch > 85.0f) headLook.targetPitch = 85.0f;
        if (headLook.targetPitch < -85.0f) headLook.targetPitch = -85.0f;
    }
    
    // Smoothly interpolate head look to target (always active for smooth return-to-center)
    headLook.yaw = headLook.yaw * headLook.smoothing + headLook.targetYaw * (1.0f - headLook.smoothing);
    headLook.pitch = headLook.pitch * headLook.smoothing + headLook.targetPitch * (1.0f - headLook.smoothing);
    
    // Return head to center when not actively looking around (F key while in cockpit)
    if (IsKeyPressed(KEY_F) && cameraMode == FLIGHT_CAMERA_COCKPIT) {
        headLook.targetYaw = 0.0f;
        headLook.targetPitch = 0.0f;
    }
    
    // Reset FOV to default (G key while in cockpit)
    if (IsKeyPressed(KEY_G) && cameraMode == FLIGHT_CAMERA_COCKPIT) {
        fovConfig.resetCockpitFOV();
    }
    
    // Force visualization controls
    if (IsKeyPressed(KEY_V)) {
        forceViz.showForces = !forceViz.showForces;
    }
    
    if (IsKeyPressed(KEY_B)) {
        forceViz.showMoments = !forceViz.showMoments;
    }
    
    if (IsKeyPressed(KEY_N)) {
        forceViz.showLabels = !forceViz.showLabels;
    }
    
    // Adjust force visualization scale with +/- keys
    if (IsKeyPressed(KEY_KP_ADD) || IsKeyPressed(KEY_EQUAL)) {
        forceViz.forceScale *= 1.5f;
        forceViz.momentScale *= 1.5f;
    }
    
    if (IsKeyPressed(KEY_KP_SUBTRACT) || IsKeyPressed(KEY_MINUS)) {
        forceViz.forceScale /= 1.5f;
        forceViz.momentScale /= 1.5f;
        
        // Prevent scale from becoming too small
        if (forceViz.forceScale < 0.001f) forceViz.forceScale = 0.001f;
        if (forceViz.momentScale < 0.001f) forceViz.momentScale = 0.001f;
    }
    
    // Mouse camera control for external view
    if (cameraMode == FLIGHT_CAMERA_EXTERNAL && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 mousePos = GetMousePosition();
        Vector2 mouseDelta = { mousePos.x - mouseLastPos.x, mousePos.y - mouseLastPos.y };
        
        cameraYaw += mouseDelta.x * 0.5f;
        cameraPitch += mouseDelta.y * 0.5f;
        
        // Clamp pitch
        if (cameraPitch > 89.0f) cameraPitch = 89.0f;
        if (cameraPitch < -89.0f) cameraPitch = -89.0f;
    }
    
    mouseLastPos = GetMousePosition();
    
    // Mouse wheel FOV control for cockpit camera
    if (cameraMode == FLIGHT_CAMERA_COCKPIT) {
        float wheelMove = GetMouseWheelMove();
        if (wheelMove != 0.0f) {
            fovConfig.adjustCockpitFOV(wheelMove);
        }
    }
    
    // Mouse wheel for distance control in external view
    if (cameraMode == FLIGHT_CAMERA_EXTERNAL) {
        float wheel = GetMouseWheelMove();
        cameraDistance -= wheel * 20.0f;
        if (cameraDistance < 20.0f) cameraDistance = 20.0f;
        if (cameraDistance > 500.0f) cameraDistance = 500.0f;
    }
}

void FlightSimulator::draw()
{
    BeginDrawing();
    
    // Sky gradient background
    ClearBackground((Color){135, 206, 235, 255}); // Sky blue
    
    BeginMode3D(camera);

    // Draw enhanced ground environment
    drawEnvironment();

    // Draw aircraft
    drawAircraft();

    // Draw flight path
    drawFlightPath();
    
    // Draw force vectors if enabled
    drawForceVectors();

    EndMode3D();

    // Draw enhanced HUD
    drawHUD();

    EndDrawing();
}

void FlightSimulator::drawAircraft()
{
    Vector3 pos = { (float)engineState.X, (float)(-engineState.Z), (float)engineState.Y };

    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);

    // Map body axes (X-forward, Y-right, Z-down) to world axes (X-north, Y-up, Z-east)
    Vector3 forward = { (float)R[0][0], (float)(-R[2][0]), (float)R[1][0] };
    Vector3 right = { (float)R[0][1], (float)(-R[2][1]), (float)R[1][1] };
    Vector3 up = { (float)(-R[0][2]), (float)R[2][2], (float)(-R[1][2]) };

    forward = Vector3Normalize(forward);
    right = Vector3Normalize(right);
    up = Vector3Normalize(up);
    
    if (hasCustomModel) {
        // Use custom 3D model with proper orientation mapping
        // Swap right and up vectors to correct the 90-degree roll offset issue
        Matrix orientation = MatrixIdentity();
        orientation.m0 = forward.x; orientation.m1 = forward.y; orientation.m2 = forward.z; orientation.m3 = 0.0f;
        orientation.m4 = up.x;     orientation.m5 = up.y;     orientation.m6 = up.z;     orientation.m7 = 0.0f;
        orientation.m8 = right.x;   orientation.m9 = right.y;   orientation.m10 = right.z;   orientation.m11 = 0.0f;
        orientation.m12 = 0.0f;     orientation.m13 = 0.0f;     orientation.m14 = 0.0f;     orientation.m15 = 1.0f;

        aircraftModel.transform = MatrixMultiply(orientation, aircraftModelBaseTransform);

        DrawModel(aircraftModel, pos, customModelScale, WHITE);
        
        // Still draw orientation indicators for reference
        float scale = 15.0f;
        
        Vector3 forwardIndicator = { pos.x + forward.x * scale, 
                                     pos.y + forward.y * scale, 
                                     pos.z + forward.z * scale };
        DrawLine3D(pos, forwardIndicator, GREEN); // Green = Forward direction
        
        Vector3 upIndicator = { pos.x + up.x * scale * 0.7f, 
                                pos.y + up.y * scale * 0.7f, 
                                pos.z + up.z * scale * 0.7f };
        DrawLine3D(pos, upIndicator, BLUE); // Blue = Up direction
    } else {
        // Use original basic aircraft representation
        float scale = 8.0f;
        
        // Draw main fuselage (along forward direction)
        Vector3 nose = { pos.x + forward.x * scale * 1.5f, 
                         pos.y + forward.y * scale * 1.5f, 
                         pos.z + forward.z * scale * 1.5f };
        Vector3 tail = { pos.x - forward.x * scale * 1.5f, 
                         pos.y - forward.y * scale * 1.5f, 
                         pos.z - forward.z * scale * 1.5f };
        
        // Draw fuselage as line (main body)
        DrawLine3D(nose, tail, RED);
        DrawSphere(nose, 1.5f, DARKGRAY); // Nose
        DrawSphere(pos, 2.0f, RED);       // Center body
        
        // Draw wings (perpendicular to forward, along right direction)
        Vector3 leftWing = { pos.x - right.x * scale * 2.0f, 
                             pos.y - right.y * scale * 2.0f, 
                             pos.z - right.z * scale * 2.0f };
        Vector3 rightWing = { pos.x + right.x * scale * 2.0f, 
                              pos.y + right.y * scale * 2.0f, 
                              pos.z + right.z * scale * 2.0f };
        
        DrawLine3D(leftWing, rightWing, GRAY);
        DrawSphere(leftWing, 1.0f, GRAY);
        DrawSphere(rightWing, 1.0f, GRAY);
        
        // Draw vertical stabilizer (tail fin, along up direction)
    Vector3 tailTop = { tail.x + up.x * scale * 0.8f, 
                tail.y + up.y * scale * 0.8f, 
                tail.z + up.z * scale * 0.8f };
        
        DrawLine3D(tail, tailTop, GRAY);
        
        // Draw attitude reference lines to clearly show orientation
        Vector3 forwardIndicator = { pos.x + forward.x * scale * 3.0f, 
                                     pos.y + forward.y * scale * 3.0f, 
                                     pos.z + forward.z * scale * 3.0f };
        DrawLine3D(pos, forwardIndicator, GREEN); // Green = Forward direction
        
    Vector3 upIndicator = { pos.x + up.x * scale * 2.0f, 
                pos.y + up.y * scale * 2.0f, 
                pos.z + up.z * scale * 2.0f };
        DrawLine3D(pos, upIndicator, BLUE); // Blue = Up direction
    }
}

void FlightSimulator::drawFlightPath()
{
    if (flightPath.size() < 2) return;

    for (size_t i = 1; i < flightPath.size(); i++) {
        DrawLine3D(flightPath[i-1], flightPath[i], BLUE);
    }
}

void FlightSimulator::drawEnvironment()
{
    // Enhanced ground plane with texture-like appearance
    DrawPlane((Vector3){ 0, 0, 0 }, (Vector2){ 5000, 5000 }, (Color){34, 139, 34, 255}); // Forest green
    
    // Major grid lines every 1000m (white)
    for (int i = -2500; i <= 2500; i += 1000) {
        if (i == 0) continue; // Skip center lines
        DrawLine3D((Vector3){(float)i, 0, -2500}, (Vector3){(float)i, 0, 2500}, WHITE);
        DrawLine3D((Vector3){-2500, 0, (float)i}, (Vector3){2500, 0, (float)i}, WHITE);
    }
    
    // Center axes (more prominent)
    DrawLine3D((Vector3){0, 0, -2500}, (Vector3){0, 0, 2500}, RED);      // North-South (Red)
    DrawLine3D((Vector3){-2500, 0, 0}, (Vector3){2500, 0, 0}, GREEN);    // East-West (Green)
    
    // Minor grid lines every 500m (gray)
    for (int i = -2500; i <= 2500; i += 500) {
        if (i % 1000 == 0) continue; // Skip major grid lines
        DrawLine3D((Vector3){(float)i, 0, -2500}, (Vector3){(float)i, 0, 2500}, LIGHTGRAY);
        DrawLine3D((Vector3){-2500, 0, (float)i}, (Vector3){2500, 0, (float)i}, LIGHTGRAY);
    }
    
    // Landmark buildings/towers for reference
    Vector3 buildingPositions[] = {
        {500, 0, 500},    // Southeast
        {-500, 0, 500},   // Southwest  
        {500, 0, -500},   // Northeast
        {-500, 0, -500},  // Northwest
        {0, 0, 1000},     // North
        {0, 0, -1000},    // South
        {1000, 0, 0},     // East
        {-1000, 0, 0}     // West
    };
    
    Color buildingColors[] = {BLUE, RED, YELLOW, PURPLE, ORANGE, PINK, BROWN, DARKGRAY};
    
    for (int i = 0; i < 8; i++) {
        // Draw building base
        DrawCube(buildingPositions[i], 20, 50, 20, buildingColors[i]);
        DrawCubeWires(buildingPositions[i], 20, 50, 20, BLACK);
        
        // Draw antenna/spire on top
        Vector3 spirePos = {buildingPositions[i].x, buildingPositions[i].y + 35, buildingPositions[i].z};
        DrawCylinder(spirePos, 1, 1, 20, 8, DARKGRAY);
    }
    
    // Runway strips for additional reference
    // Draw runway (black asphalt)
    DrawCube((Vector3){0, 0.05f, -1450}, 400, 0.1f, 100, (Color){32, 32, 32, 255});
    
    // Runway center line (yellow dashes)
    for (int i = -180; i <= 180; i += 40) {
        DrawCube((Vector3){(float)i, 0.11f, -1450}, 20, 0.02f, 2, YELLOW);
    }
    
    // Runway edge lines (white)
    DrawLine3D((Vector3){-200, 0.11f, -1400}, (Vector3){200, 0.11f, -1400}, WHITE);
    DrawLine3D((Vector3){-200, 0.11f, -1500}, (Vector3){200, 0.11f, -1500}, WHITE);
    
    // Distance markers every 1000m from origin
    for (int dist = 1000; dist <= 3000; dist += 1000) {
        // North marker
        DrawSphere((Vector3){0, 5, (float)dist}, 3, BLUE);
        DrawSphere((Vector3){0, 5, (float)-dist}, 3, BLUE);
        // East marker  
        DrawSphere((Vector3){(float)dist, 5, 0}, 3, RED);
        DrawSphere((Vector3){(float)-dist, 5, 0}, 3, RED);
    }
    
    // Origin marker (large)
    DrawSphere((Vector3){0, 2, 0}, 5, GOLD);
    DrawCylinder((Vector3){0, 10, 0}, 2, 2, 20, 6, GOLD);
}

void FlightSimulator::drawHUD()
{
    // Enhanced flight data display
    double rollRad, pitchRad, yawRad;
    QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);
    
    // Convert to aviation standard angles
    double rollDeg = rollRad * 180.0 / M_PI;
    double pitchDeg = pitchRad * 180.0 / M_PI;
    double headingDeg = yawRad * 180.0 / M_PI;
    
    // Normalize heading to 0-360 degrees
    if (headingDeg < 0) headingDeg += 360.0;

    char text[256];
    
    // Left side - Primary flight data
    DrawText("FLIGHT DATA", 10, 10, 16, YELLOW);
    
    double displayedSimTime = hasSimulationEventData ? lastSimulationTimeEvent
                                                    : simulationEngine.getSimulationTime();

    sprintf(text, "Sim Time: %.1fs", displayedSimTime);
    DrawText(text, 10, 30, 18, WHITE);

    sprintf(text, "Fixed Step: %.3f s", lastFixedStepDelta);
    DrawText(text, 10, 50, 18, WHITE);

    sprintf(text, "Altitude: %.0fm", -engineState.Z);
    DrawText(text, 10, 70, 18, WHITE);

    sprintf(text, "Airspeed: %.1f m/s", currentAirspeed);
    DrawText(text, 10, 90, 18, WHITE);
    
    sprintf(text, "Climb Rate: %.1f m/s", currentClimbRate);
    DrawText(text, 10, 110, 18, currentClimbRate > 0 ? GREEN : (currentClimbRate < -5 ? RED : WHITE));

    sprintf(text, "AoA: %.2f°", currentAoA * 180.0 / M_PI);
    DrawText(text, 10, 130, 18, fabs(currentAoA) > 0.3 ? RED : WHITE);

    sprintf(text, "Sideslip: %.2f°", currentBeta * 180.0 / M_PI);
    DrawText(text, 10, 150, 18, fabs(currentBeta) > 0.1 ? YELLOW : WHITE);
    
    // Attitude data (Aviation Standard Format)
    DrawText("ATTITUDE:", 10, 170, 14, YELLOW);
    
    sprintf(text, "Roll: %+.1f°", rollDeg);
    Color rollColor = (fabs(rollDeg) > 45.0) ? RED : (fabs(rollDeg) > 20.0) ? YELLOW : WHITE;
    DrawText(text, 10, 190, 18, rollColor);

    sprintf(text, "Pitch: %+.1f°", pitchDeg);
    Color pitchColor = (fabs(pitchDeg) > 30.0) ? RED : (fabs(pitchDeg) > 15.0) ? YELLOW : WHITE;
    DrawText(text, 10, 210, 18, pitchColor);

    sprintf(text, "Heading: %03.0f°", headingDeg);
    DrawText(text, 10, 230, 18, WHITE);
    
    // Flight path data
    DrawText("FLIGHT PATH:", 10, 250, 14, YELLOW);
    
    // Calculate ground speed and track
    double groundSpeed = sqrt(engineState.Vx * engineState.Vx + engineState.Vy * engineState.Vy);
    double track = atan2(engineState.Vy, engineState.Vx) * 180.0 / M_PI;
    if (track < 0) track += 360.0;
    
    sprintf(text, "Ground Speed: %.1f m/s", groundSpeed);
    DrawText(text, 10, 270, 12, WHITE);
    
    sprintf(text, "Track: %03.0f°", track);
    Color trackColor = (fabs(track - headingDeg) > 5.0) ? YELLOW : WHITE;
    DrawText(text, 10, 285, 12, trackColor);

    // Controls
    DrawText("FLIGHT CONTROLS:", 10, 305, 14, YELLOW);
    DrawText("W/S - Elevator (W=Down, S=Up)", 10, 325, 12, WHITE);
    DrawText("A/D - Aileron (A=Left, D=Right)", 10, 340, 12, WHITE);
    DrawText("Q/E - Rudder (Yaw)", 10, 355, 12, WHITE);
    DrawText("Shift/Ctrl - Throttle", 10, 370, 12, WHITE);
    
    DrawText("CAMERA CONTROLS:", 10, 390, 14, YELLOW);
    DrawText("SPACE - Pause/Resume", 10, 410, 12, WHITE);
    DrawText("R - Reset Simulation", 10, 425, 12, WHITE);
    DrawText("C - Switch Camera", 10, 440, 12, WHITE);
    DrawText("H - Head Look On/Off (Cockpit)", 10, 435, 12, WHITE);
    DrawText("F - Center Head (Cockpit)", 10, 450, 12, WHITE);
    DrawText("G - Reset FOV (Cockpit)", 10, 465, 12, WHITE);
    DrawText("Left Click+Drag - Camera/Head", 10, 480, 12, WHITE);
    DrawText("Mouse Wheel - Zoom/FOV", 10, 495, 12, WHITE);
    DrawText("L - Load Aircraft Model", 10, 510, 12, WHITE);
    DrawText("M - Toggle Model Display", 10, 525, 12, WHITE);
    
    DrawText("FORCE VISUALIZATION:", 10, 545, 14, YELLOW);
    DrawText("V - Toggle Forces", 10, 565, 12, forceViz.showForces ? GREEN : WHITE);
    DrawText("B - Toggle Moments", 10, 580, 12, forceViz.showMoments ? GREEN : WHITE);
    DrawText("N - Toggle Labels", 10, 595, 12, forceViz.showLabels ? GREEN : WHITE);
    DrawText("+/- - Scale Vectors", 10, 610, 12, WHITE);
    
    // Control status display (right side)
    DrawText("CONTROL STATUS:", GetScreenWidth() - 200, 50, 14, YELLOW);
    
    sprintf(text, "Aileron: %.2f", controls.aileron);
    Color aileronColor = (fabs(controls.aileron) > 0.1f) ? GREEN : WHITE;
    DrawText(text, GetScreenWidth() - 200, 70, 12, aileronColor);
    
    sprintf(text, "Elevator: %.2f", controls.elevator);
    Color elevatorColor = (fabs(controls.elevator) > 0.1f) ? GREEN : WHITE;
    DrawText(text, GetScreenWidth() - 200, 85, 12, elevatorColor);
    
    sprintf(text, "Rudder: %.2f", controls.rudder);
    Color rudderColor = (fabs(controls.rudder) > 0.1f) ? GREEN : WHITE;
    DrawText(text, GetScreenWidth() - 200, 100, 12, rudderColor);
    
    sprintf(text, "Throttle: %.2f", controls.throttle);
    Color throttleColor = (controls.throttle > 0.1f) ? GREEN : WHITE;
    DrawText(text, GetScreenWidth() - 200, 115, 12, throttleColor);

    if (activeGamepadIndex >= 0) {
        sprintf(text, "Gamepad %d: %s", activeGamepadIndex, activeGamepadName.c_str());
        DrawText(text, GetScreenWidth() - 200, 135, 12, LIGHTGRAY);

        sprintf(text, "Joy Roll: %.2f", lastGamepadRoll);
        DrawText(text, GetScreenWidth() - 200, 150, 12, LIGHTGRAY);

        sprintf(text, "Joy Pitch: %.2f", lastGamepadPitch);
        DrawText(text, GetScreenWidth() - 200, 165, 12, LIGHTGRAY);

        sprintf(text, "Axes: %d", lastGamepadAxisCount);
        DrawText(text, GetScreenWidth() - 200, 180, 12, LIGHTGRAY);

        int displayAxes = std::min<int>((int)lastGamepadAxes.size(), lastGamepadAxisCount);
        for (int i = 0; i < displayAxes; ++i) {
            sprintf(text, "A%d: %.2f", i, lastGamepadAxes[i]);
            DrawText(text, GetScreenWidth() - 200, 195 + i * 12, 12, LIGHTGRAY);
        }
    } else {
        DrawText("Gamepad: Not detected", GetScreenWidth() - 200, 135, 12, RED);
    }
    
    // Force visualization status and info
    int forceInfoY = 200;
    DrawText("FORCE VISUALIZATION:", GetScreenWidth() - 200, forceInfoY, 14, YELLOW);
    
    sprintf(text, "Forces: %s (%d)", forceViz.showForces ? "ON" : "OFF", (int)forceViz.forces.size());
    DrawText(text, GetScreenWidth() - 200, forceInfoY + 20, 12, forceViz.showForces ? GREEN : WHITE);
    
    sprintf(text, "Moments: %s (%d)", forceViz.showMoments ? "ON" : "OFF", (int)forceViz.moments.size());
    DrawText(text, GetScreenWidth() - 200, forceInfoY + 35, 12, forceViz.showMoments ? GREEN : WHITE);
    
    sprintf(text, "Force Scale: %.4f", forceViz.forceScale);
    DrawText(text, GetScreenWidth() - 200, forceInfoY + 50, 12, WHITE);
    
    sprintf(text, "Moment Scale: %.4f", forceViz.momentScale);
    DrawText(text, GetScreenWidth() - 200, forceInfoY + 65, 12, WHITE);
    
    // Show force magnitudes if forces are visible
    if (forceViz.showForces && !forceViz.forces.empty()) {
        DrawText("FORCE MAGNITUDES:", GetScreenWidth() - 200, forceInfoY + 85, 12, YELLOW);
        int lineY = forceInfoY + 100;
        
        for (size_t i = 0; i < forceViz.forces.size() && i < 5; i++) { // Show max 5 forces
            const auto& force = forceViz.forces[i];
            float magnitude = sqrtf(force.force.x * force.force.x + force.force.y * force.force.y + force.force.z * force.force.z);
            sprintf(text, "%s: %.1fN", force.label.c_str(), magnitude);
            DrawText(text, GetScreenWidth() - 200, lineY, 10, force.color);
            lineY += 12;
        }
    }
    
    // Show moment magnitudes if moments are visible
    if (forceViz.showMoments && !forceViz.moments.empty()) {
        int momentY = forceInfoY + 200;
        DrawText("MOMENT MAGNITUDES:", GetScreenWidth() - 200, momentY, 12, YELLOW);
        int lineY = momentY + 15;
        
        for (size_t i = 0; i < forceViz.moments.size() && i < 3; i++) { // Show max 3 moments
            const auto& moment = forceViz.moments[i];
            float magnitude = sqrtf(moment.force.x * moment.force.x + moment.force.y * moment.force.y + moment.force.z * moment.force.z);
            sprintf(text, "%s: %.1fN·m", moment.label.c_str(), magnitude);
            DrawText(text, GetScreenWidth() - 200, lineY, 10, moment.color);
            lineY += 12;
        }
    }
    
    // Camera mode indicator with FOV and head tracking info
    sprintf(text, "Camera: %s (FOV: %.1f°)", 
            cameraMode == FLIGHT_CAMERA_COCKPIT ? "COCKPIT" : "EXTERNAL", 
            camera.fovy);
    DrawText(text, GetScreenWidth() - 280, 10, 16, YELLOW);
    
    // Head tracking status (cockpit only)
    if (cameraMode == FLIGHT_CAMERA_COCKPIT) {
        sprintf(text, "Head Look: %s", mouseHeadLookEnabled ? "ON" : "OFF");
        DrawText(text, GetScreenWidth() - 280, 30, 14, mouseHeadLookEnabled ? GREEN : WHITE);
        
        // FOV zoom info
        float defaultFOV = fovConfig.calculateVerticalFOV();
        if (fabsf(fovConfig.cockpitFOV - defaultFOV) > 1.0f) {
            sprintf(text, "FOV: %.1f° (Default: %.1f°)", fovConfig.cockpitFOV, defaultFOV);
            DrawText(text, GetScreenWidth() - 280, 45, 12, {0, 255, 255, 255}); // Cyan color
        }
        
        if (mouseHeadLookEnabled || fabsf(headLook.yaw) > 1.0f || fabsf(headLook.pitch) > 1.0f) {
            sprintf(text, "Head: Yaw %+.1f° Pitch %+.1f°", headLook.yaw, headLook.pitch);
            DrawText(text, GetScreenWidth() - 280, 60, 12, WHITE);
        }
    }

    if (!isSimulationRunning) {
        DrawText("SIMULATION PAUSED", GetScreenWidth()/2 - 100, GetScreenHeight()/2, 24, RED);
    }
    
    // Draw flight path marker and attitude indicator
    drawFlightPathMarker();
    drawAttitudeIndicator();
}

void FlightSimulator::updateCameraSystem()
{
    Vector3 aircraftPos = { (float)engineState.X, (float)(-engineState.Z), (float)engineState.Y };
    
    // Update camera FOV based on camera mode
    if (cameraMode == FLIGHT_CAMERA_COCKPIT) {
        camera.fovy = fovConfig.getCockpitFOV(); // Use dynamic cockpit FOV
    } else {
        camera.fovy = fovConfig.calculateVerticalFOV(); // Use calculated FOV for external
    }
    
    if (cameraMode == FLIGHT_CAMERA_COCKPIT) {
        // Cockpit camera with head tracking support
        double R[3][3];
        QuaternionOperations::toMatrix(engineState.q, R);
        
        // Base cockpit position relative to aircraft center (body coordinates)
        float cockpitX = headLook.headOffset.x;  // Forward
        float cockpitY = headLook.headOffset.y;  // Right  
        float cockpitZ = headLook.headOffset.z;  // Down
        
        // Transform base cockpit position from aircraft body frame to world frame
        Vector3 cockpitOffset;
        cockpitOffset.x = (float)(R[0][0] * cockpitX + R[0][1] * cockpitY + R[0][2] * cockpitZ);
        cockpitOffset.y = (float)(-R[2][0] * cockpitX - R[2][1] * cockpitY - R[2][2] * cockpitZ);
        cockpitOffset.z = (float)(R[1][0] * cockpitX + R[1][1] * cockpitY + R[1][2] * cockpitZ);
        
        // Set camera position at cockpit location
        camera.position.x = aircraftPos.x + cockpitOffset.x;
        camera.position.y = aircraftPos.y + cockpitOffset.y;
        camera.position.z = aircraftPos.z + cockpitOffset.z;
        
        // Calculate head tracking rotation
        // Convert head yaw/pitch to radians  
        float headYawRad = headLook.yaw * M_PI / 180.0f;
        float headPitchRad = headLook.pitch * M_PI / 180.0f;
        
        // Create head rotation matrix (yaw around aircraft Z-axis, pitch around head's side axis)
        // Head yaw rotates around aircraft's up vector
        // Head pitch rotates around head's side vector (perpendicular to forward and up)
        
        // Aircraft forward direction (body frame X-axis -> world frame)
        Vector3 aircraftForward;
        aircraftForward.x = (float)R[0][0];
        aircraftForward.y = (float)(-R[2][0]);  
        aircraftForward.z = (float)R[1][0];
        
        // Aircraft right direction (body frame Y-axis -> world frame)  
        Vector3 aircraftRight;
        aircraftRight.x = (float)R[0][1];
        aircraftRight.y = (float)(-R[2][1]);
        aircraftRight.z = (float)R[1][1];
        
        // Aircraft up direction (body frame Z-axis -> world frame, inverted)
        Vector3 aircraftUp;
        aircraftUp.x = (float)(-R[0][2]);
        aircraftUp.y = (float)(R[2][2]);
        aircraftUp.z = (float)(-R[1][2]);
        
        // Apply head yaw rotation around aircraft up vector
        Vector3 headForward;
        float cosYaw = cosf(headYawRad);
        float sinYaw = sinf(headYawRad);
        
        // Rotate forward vector by head yaw around up vector
        headForward.x = aircraftForward.x * cosYaw + aircraftRight.x * sinYaw;
        headForward.y = aircraftForward.y * cosYaw + aircraftRight.y * sinYaw;  
        headForward.z = aircraftForward.z * cosYaw + aircraftRight.z * sinYaw;
        
        // Recalculate head right vector (perpendicular to head forward and aircraft up)
        Vector3 headRight = Vector3CrossProduct(headForward, aircraftUp);
        headRight = Vector3Normalize(headRight);
        
        // Apply head pitch rotation around head right vector
        Vector3 finalForward;
        float cosPitch = cosf(headPitchRad);
        float sinPitch = sinf(headPitchRad);
        
        // Rotate head forward by pitch around head right vector
        Vector3 headUp = Vector3CrossProduct(headRight, headForward);
        headUp = Vector3Normalize(headUp);
        
        finalForward.x = headForward.x * cosPitch + headUp.x * sinPitch;
        finalForward.y = headForward.y * cosPitch + headUp.y * sinPitch;
        finalForward.z = headForward.z * cosPitch + headUp.z * sinPitch;
        
        // Final head up vector after pitch rotation
        Vector3 finalUp;
        finalUp.x = -headForward.x * sinPitch + headUp.x * cosPitch;
        finalUp.y = -headForward.y * sinPitch + headUp.y * cosPitch;
        finalUp.z = -headForward.z * sinPitch + headUp.z * cosPitch;
        
        // Set target point far ahead in final look direction
        camera.target.x = camera.position.x + finalForward.x * 1000.0f;
        camera.target.y = camera.position.y + finalForward.y * 1000.0f;
        camera.target.z = camera.position.z + finalForward.z * 1000.0f;
        
        // Set camera up vector
        camera.up = finalUp;
        
    } else {
        // External view with mouse control (unchanged)
        float yawRad = cameraYaw * M_PI / 180.0f;
        float pitchRad = cameraPitch * M_PI / 180.0f;
        
        Vector3 offset;
        offset.x = cameraDistance * cosf(pitchRad) * cosf(yawRad);
        offset.y = cameraDistance * sinf(pitchRad);
        offset.z = cameraDistance * cosf(pitchRad) * sinf(yawRad);
        
        camera.position = (Vector3){ aircraftPos.x + offset.x, 
                                     aircraftPos.y + offset.y, 
                                     aircraftPos.z + offset.z };
        camera.target = aircraftPos;
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    }
}

void FlightSimulator::drawFlightPathMarker()
{
    if (currentAirspeed < 1.0) return;
    
    int centerX = GetScreenWidth() / 2;
    int centerY = GetScreenHeight() / 2;
    
    // Calculate velocity vector in body coordinates
    double vx_body = bodyState.u;
    double vy_body = bodyState.v;
    double vz_body = bodyState.w;
    
    // Project velocity vector to screen - using all components for future expansion
    // This is a simplified projection - in reality you'd want proper 3D to 2D projection
    if (cameraMode == FLIGHT_CAMERA_COCKPIT && currentAirspeed > 5.0) {
        float offsetX = (float)(vy_body * 100.0 / currentAirspeed);
        float offsetY = (float)(-vz_body * 100.0 / currentAirspeed);
        // vx_body represents forward velocity, affects marker size/visibility
        float markerSize = 15.0f + (float)(vx_body * 5.0 / currentAirspeed);
        
        int markerX = centerX + (int)offsetX;
        int markerY = centerY + (int)offsetY;
        
        // Draw flight path marker (velocity vector) with dynamic size
        DrawCircleLines(markerX, markerY, (int)markerSize, GREEN);
        DrawLine(markerX - 20, markerY, markerX - 10, markerY, GREEN);
        DrawLine(markerX + 10, markerY, markerX + 20, markerY, GREEN);
        DrawLine(markerX, markerY - 20, markerX, markerY - 10, GREEN);
        
        DrawText("FPV", markerX + 20, markerY - 10, 12, GREEN);
    }
}

void FlightSimulator::drawAttitudeIndicator()
{
    if (cameraMode != FLIGHT_CAMERA_COCKPIT) return;
    
    double rollRad, pitchRad, yawRad;
    QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);
    
    int centerX = GetScreenWidth() - 150;
    int centerY = 150;
    int radius = 80;
    
    // Draw outer frame
    DrawCircle(centerX, centerY, radius + 5, BLACK);
    DrawCircleLines(centerX, centerY, radius + 5, WHITE);
    DrawCircle(centerX, centerY, radius, BLACK);
    
    // Convert to degrees for calculations
    float pitchDeg = (float)(pitchRad * 180.0 / M_PI);
    float rollDeg = (float)(rollRad * 180.0 / M_PI);
    
    // Handle gimbal lock and extreme attitudes
    // Clamp pitch for display stability (real aircraft rarely exceed ±70°)
    float displayPitchDeg = pitchDeg;
    if (displayPitchDeg > 85.0f) displayPitchDeg = 85.0f;
    if (displayPitchDeg < -85.0f) displayPitchDeg = -85.0f;
    
    // For extreme pitch attitudes, stabilize roll display
    float displayRollRad = rollRad;
    if (fabsf(pitchDeg) > 75.0f) {
        // In extreme pitch, limit roll rotation to prevent gimbal lock visual artifacts
        float rollStabilizationFactor = (90.0f - fabsf(pitchDeg)) / 15.0f; // Scale from 1.0 to 0.0
        rollStabilizationFactor = fmaxf(0.0f, fminf(1.0f, rollStabilizationFactor));
        displayRollRad *= rollStabilizationFactor;
    }
    
    // Scale: 1 degree = 2.5 pixels for realistic appearance
    float pitchPixelsPerDegree = 2.5f;
    float horizonOffset = displayPitchDeg * pitchPixelsPerDegree;
    
    // Clamp extreme horizon offset values
    if (horizonOffset > radius * 1.5f) horizonOffset = radius * 1.5f;
    if (horizonOffset < -radius * 1.5f) horizonOffset = -radius * 1.5f;
    
    // Draw sky and ground with proper masking
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                // Apply roll rotation to coordinates
                float cosRoll = cosf(displayRollRad);  // Remove negative sign for correct sky/ground rotation
                float sinRoll = sinf(displayRollRad);
                float rotatedY = x * sinRoll + y * cosRoll;
                
                // Determine sky vs ground based on rotated position relative to horizon
                if (rotatedY < horizonOffset) {
                    // SKY - above horizon line
                    DrawPixel(centerX + x, centerY + y, (Color){135, 206, 235, 255}); // Sky blue
                } else {
                    // GROUND - below horizon line  
                    DrawPixel(centerX + x, centerY + y, (Color){101, 67, 33, 255}); // Earth brown
                }
            }
        }
    }
    
    // White horizon line removed - only sky/ground colors show the horizon
    
    // Draw pitch ladder removed - only sky/ground display
    
    // Draw aircraft reference symbol (FIXED in center)
    // Wing reference bars
    DrawRectangle(centerX - 30, centerY - 2, 12, 4, YELLOW);
    DrawRectangle(centerX + 18, centerY - 2, 12, 4, YELLOW);
    
    // Center fuselage reference
    DrawRectangle(centerX - 8, centerY - 1, 16, 2, YELLOW);
    DrawCircle(centerX, centerY, 3, YELLOW);
    
    // Vertical reference line
    DrawLine(centerX, centerY - 8, centerX, centerY + 8, YELLOW);
    
    // FLIGHT PATH MARKER - Shows where aircraft is actually going
    // Calculate velocity vector in aircraft body frame
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);
    
    // Transform world velocity to body frame
    double vx_body = R[0][0] * engineState.Vx + R[1][0] * engineState.Vy + R[2][0] * engineState.Vz;
    double vy_body = R[0][1] * engineState.Vx + R[1][1] * engineState.Vy + R[2][1] * engineState.Vz; 
    double vz_body = R[0][2] * engineState.Vx + R[1][2] * engineState.Vy + R[2][2] * engineState.Vz;
    
    // Calculate flight path angles
    double groundSpeed = sqrt(vx_body * vx_body + vy_body * vy_body + vz_body * vz_body);
    
    if (groundSpeed > 5.0) { // Only show if aircraft is moving
        // Flight path angle (vertical) - angle between velocity and horizontal plane
        // Negative vz_body means climb in body frame (Z points down)
        double fpAngleRad = atan2(-vz_body, sqrt(vx_body * vx_body + vy_body * vy_body));
        
        // Track angle (horizontal) - sideslip component (right is positive)
        double trackAngleRad = atan2(-vy_body, vx_body);
        
        // Convert to screen coordinates relative to attitude indicator
        float fpAngleDeg = (float)(fpAngleRad * 180.0 / M_PI);
        float trackAngleDeg = (float)(trackAngleRad * 180.0 / M_PI);
        
        // Scale flight path angles for display - INVERT vertical for screen coordinates
        float fpScreenOffset = -fpAngleDeg * pitchPixelsPerDegree;  // Negative for correct screen mapping
        float trackScreenOffset = trackAngleDeg * pitchPixelsPerDegree;
        
        // NO ROLL COMPENSATION - Simple vertical-only flight path marker
        // Flight path marker position (fixed to screen coordinates, no roll rotation)
        float fpMarkerX = centerX + trackScreenOffset;  // Horizontal sideslip only
        float fpMarkerY = centerY + fpScreenOffset;     // Vertical flight path angle only
        
        // Only draw if within attitude indicator circle
        float distFromCenter = sqrt((fpMarkerX - centerX) * (fpMarkerX - centerX) + (fpMarkerY - centerY) * (fpMarkerY - centerY));
        if (distFromCenter <= radius - 5) {
            // Draw flight path marker symbol (circle with cross)
            Color fpColor = WHITE;
            
            // Flight path marker circle
            DrawCircleLines((int)fpMarkerX, (int)fpMarkerY, 8, fpColor);
            
            // Cross lines (showing actual flight direction)
            DrawLine((int)fpMarkerX - 15, (int)fpMarkerY, (int)fpMarkerX - 8, (int)fpMarkerY, fpColor);  // Left
            DrawLine((int)fpMarkerX + 8, (int)fpMarkerY, (int)fpMarkerX + 15, (int)fpMarkerY, fpColor);   // Right
            DrawLine((int)fpMarkerX, (int)fpMarkerY - 15, (int)fpMarkerX, (int)fpMarkerY - 8, fpColor);   // Top
            DrawLine((int)fpMarkerX, (int)fpMarkerY + 8, (int)fpMarkerX, (int)fpMarkerY + 5, fpColor);    // Bottom (shorter)
            
            // Small center dot
            DrawCircle((int)fpMarkerX, (int)fpMarkerY, 2, fpColor);
        }
        
        // Draw flight path angle and track angle indicators outside the attitude indicator
        // Flight path angle indicator (vertical scale) - left side
        int fpaX = centerX - radius - 25;
        int fpaY = centerY;
        float fpaIndicatorY = fpaY - fpAngleDeg * 2.0f; // Scale for display
        
        // Clamp to reasonable display range
        if (fpaIndicatorY >= centerY - radius && fpaIndicatorY <= centerY + radius) {
            DrawLine(fpaX - 5, (int)fpaIndicatorY, fpaX + 5, (int)fpaIndicatorY, WHITE);
            DrawLine(fpaX, (int)fpaIndicatorY - 3, fpaX, (int)fpaIndicatorY + 3, WHITE);
            
            // Show FPA value
            char fpaText[16];
            sprintf(fpaText, "%.1f°", fpAngleDeg);
            DrawText(fpaText, fpaX - 20, (int)fpaIndicatorY - 6, 8, WHITE);
        }
        
        // Track angle indicator (horizontal scale) - bottom
        int taX = centerX;
        int taY = centerY + radius + 25;
        float taIndicatorX = taX + trackAngleDeg * 2.0f; // Scale for display
        
        // Clamp to reasonable display range
        if (taIndicatorX >= centerX - radius && taIndicatorX <= centerX + radius) {
            DrawLine((int)taIndicatorX, taY - 5, (int)taIndicatorX, taY + 5, WHITE);
            DrawLine((int)taIndicatorX - 3, taY, (int)taIndicatorX + 3, taY, WHITE);
            
            // Show track angle value
            char taText[16];
            sprintf(taText, "%.1f°", trackAngleDeg);
            DrawText(taText, (int)taIndicatorX - 12, taY + 8, 8, WHITE);
        }
    }
    
    // Roll scale (outer ring) - FIXED markers
    for (int roll = -60; roll <= 60; roll += 10) {
        if (roll == 0) continue;
        
        float angle = (float)(roll * M_PI / 180.0);
        int markLength = (abs(roll) % 30 == 0) ? 15 : 10;
        float markThickness = (abs(roll) % 30 == 0) ? 3.0f : 2.0f;
        
        Vector2 outerPoint = {
            centerX + (radius + 5) * sinf(angle),
            centerY - (radius + 5) * cosf(angle)
        };
        Vector2 innerPoint = {
            centerX + (radius + 5 - markLength) * sinf(angle),
            centerY - (radius + 5 - markLength) * cosf(angle)
        };
        
        DrawLineEx(outerPoint, innerPoint, markThickness, WHITE);
        
        // Roll angle numbers
        if (abs(roll) % 30 == 0) {
            char rollText[4];
            sprintf(rollText, "%d", abs(roll));
            Vector2 textPos = {
                centerX + (radius + 20) * sinf(angle) - 6,
                centerY - (radius + 20) * cosf(angle) - 6
            };
            DrawText(rollText, (int)textPos.x, (int)textPos.y, 10, WHITE);
        }
    }
    
    // Roll reference triangle at top (FIXED)
    Vector2 rollRef[3] = {
        {(float)centerX, (float)(centerY - radius - 8)},
        {(float)(centerX - 6), (float)(centerY - radius - 18)},
        {(float)(centerX + 6), (float)(centerY - radius - 18)}
    };
    DrawTriangle(rollRef[0], rollRef[1], rollRef[2], WHITE);
    
    // Bank angle pointer (MOVES with aircraft)
    Vector2 bankPointer[3] = {
        {(float)centerX, (float)(centerY - radius - 3)},
        {(float)(centerX - 5), (float)(centerY - radius - 12)},
        {(float)(centerX + 5), (float)(centerY - radius - 12)}
    };
    
    // Rotate bank pointer according to roll
    for (int i = 0; i < 3; i++) {
        float x = bankPointer[i].x - centerX;
        float y = bankPointer[i].y - centerY;
        bankPointer[i].x = centerX + x * cosf(-displayRollRad) - y * sinf(-displayRollRad);
        bankPointer[i].y = centerY + x * sinf(-displayRollRad) + y * cosf(-displayRollRad);
    }
    
    DrawTriangle(bankPointer[0], bankPointer[1], bankPointer[2], YELLOW);
    
    // COMPACT COCKPIT INSTRUMENTS - Purple outlined areas in your image
    
    // Convert units for cockpit instruments
    float airspeeedKnots = currentAirspeed * 1.94384f; // m/s to knots
    float altitudeFeet = -engineState.Z * 3.28084f;    // meters to feet
    float climbRateFpm = currentClimbRate * 196.85f;   // m/s to feet per minute
    
    char instText[64];
    
    // LEFT SIDE - COMPACT AIRSPEED INDICATOR (Knots)
    int asX = centerX - radius - 85;  // Left of attitude indicator
    int asY = centerY - 60;
    int asWidth = 65;
    int asHeight = 120;
    
    // Airspeed background
    DrawRectangle(asX, asY, asWidth, asHeight, Fade(BLACK, 0.85f));
    DrawRectangleLines(asX, asY, asWidth, asHeight, WHITE);
    
    // Airspeed tape marks
    for (int speed = 0; speed <= 200; speed += 20) {
        float relativeSpeed = speed - airspeeedKnots;
        if (fabsf(relativeSpeed) <= 60) {
            int y = (asY + asHeight/2) - (int)(relativeSpeed * 1.0f);
            if (y >= asY + 5 && y <= asY + asHeight - 5) {
                DrawLine(asX + asWidth - 12, y, asX + asWidth - 2, y, WHITE);
                if (speed % 40 == 0 && speed != (int)airspeeedKnots) {
                    sprintf(instText, "%d", speed);
                    DrawText(instText, asX + 5, y - 4, 8, WHITE);
                }
            }
        }
    }
    
    // Current airspeed value (center)
    sprintf(instText, "%.0f", airspeeedKnots);
    DrawRectangle(asX + asWidth - 2, (asY + asHeight/2) - 8, 25, 16, GREEN);
    DrawText(instText, asX + asWidth + 2, (asY + asHeight/2) - 6, 10, BLACK);
    DrawText("KIAS", asX + 20, asY + asHeight + 5, 8, WHITE);
    
    // RIGHT SIDE - COMPACT ALTITUDE INDICATOR (Feet)
    int altX = centerX + radius + 20;  // Right of attitude indicator  
    int altY = centerY - 60;
    int altWidth = 65;
    int altHeight = 120;
    
    // Altitude background
    DrawRectangle(altX, altY, altWidth, altHeight, Fade(BLACK, 0.85f));
    DrawRectangleLines(altX, altY, altWidth, altHeight, WHITE);
    
    // Altitude tape marks
    int baseAlt = ((int)(altitudeFeet / 200)) * 200;
    for (int alt = baseAlt - 600; alt <= baseAlt + 600; alt += 100) {
        float relativeAlt = alt - altitudeFeet;
        if (fabsf(relativeAlt) <= 400) {
            int y = (altY + altHeight/2) + (int)(relativeAlt * 0.15f);
            if (y >= altY + 5 && y <= altY + altHeight - 5) {
                DrawLine(altX + 2, y, altX + 12, y, WHITE);
                if (alt % 200 == 0 && alt != (int)altitudeFeet) {
                    if (alt >= 1000) {
                        sprintf(instText, "%.0fK", alt/1000.0f);
                    } else {
                        sprintf(instText, "%d", alt);
                    }
                    DrawText(instText, altX + 15, y - 4, 8, WHITE);
                }
            }
        }
    }
    
    // Current altitude value (center)
    sprintf(instText, "%.0f", altitudeFeet);
    DrawRectangle(altX - 25, (altY + altHeight/2) - 8, 27, 16, GREEN);
    DrawText(instText, altX - 22, (altY + altHeight/2) - 6, 10, BLACK);
    DrawText("ALT FT", altX + 15, altY + altHeight + 5, 8, WHITE);
    
    // VARIOMETER - Below altitude (Compact)
    int vsiX = altX;
    int vsiY = altY + altHeight + 20;
    int vsiWidth = 65;
    int vsiHeight = 35;
    
    DrawRectangle(vsiX, vsiY, vsiWidth, vsiHeight, Fade(BLACK, 0.85f));
    DrawRectangleLines(vsiX, vsiY, vsiWidth, vsiHeight, WHITE);
    
    // VSI scale marks (-1000 to +1000 FPM)
    for (int rate = -1000; rate <= 1000; rate += 500) {
        if (rate == 0) continue;
        float xPos = (vsiX + vsiWidth/2) + (rate / 2000.0f) * (vsiWidth * 0.7f);
        if (xPos >= vsiX + 5 && xPos <= vsiX + vsiWidth - 5) {
            DrawLine((int)xPos, vsiY + vsiHeight - 8, (int)xPos, vsiY + vsiHeight - 3, WHITE);
            sprintf(instText, "%+.0f", rate/100.0f);  // Show in hundreds
            int textW = MeasureText(instText, 6);
            DrawText(instText, (int)(xPos - textW/2), vsiY + 3, 6, WHITE);
        }
    }
    
    // Current VSI pointer
    float vsiClampedFpm = fmin(fmax(climbRateFpm, -2000), 2000);
    float vsiPointerX = (vsiX + vsiWidth/2) + (vsiClampedFpm / 2000.0f) * (vsiWidth * 0.7f);
    DrawTriangle({vsiPointerX, (float)(vsiY + vsiHeight - 3)}, 
                {vsiPointerX - 3.0f, (float)(vsiY + vsiHeight + 3)},
                {vsiPointerX + 3.0f, (float)(vsiY + vsiHeight + 3)}, GREEN);
    
    // VSI digital readout
    sprintf(instText, "%+.0f", climbRateFpm);
    Color vsiColor = (climbRateFpm > 100) ? GREEN : (climbRateFpm < -100) ? RED : WHITE;
    int vsiTextWidth = MeasureText(instText, 8);
    DrawText(instText, vsiX + vsiWidth/2 - vsiTextWidth/2, vsiY + vsiHeight/2 - 4, 8, vsiColor);
    DrawText("VSI FPM", vsiX + 15, vsiY + vsiHeight + 5, 8, WHITE);
    
    // G-METRE - Below VSI (Compact)
    int gX = centerX;
    int gY = vsiY + vsiHeight + 25;
    int gWidth = 65;
    int gHeight = 35;
    
    // Calculate G-forces from body frame acceleration (REAL METHOD)
    static double lastVx = 0.0, lastVy = 0.0, lastVz = 0.0;
    static float lastTime = 0.0f;
    float currentTime = GetTime();
    float deltaTime = currentTime - lastTime;
    
    float gForce = 1.0f; // Default 1G
    
    if (deltaTime > 0.016f) { // Update every ~60fps
        // Calculate NED accelerations first
        double ax_ned = (engineState.Vx - lastVx) / deltaTime;
        double ay_ned = (engineState.Vy - lastVy) / deltaTime;
        double az_ned = (engineState.Vz - lastVz) / deltaTime;
        
        // Transform NED accelerations to body frame using quaternion
        double rollRad, pitchRad, yawRad;
        QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);
        
        // Rotation matrix from NED to body frame
        double cosR = cos(rollRad), sinR = sin(rollRad);
        double cosP = cos(pitchRad), sinP = sin(pitchRad);  
        double cosY = cos(yawRad), sinY = sin(yawRad);
        
        // Body frame accelerations (u_dot, v_dot, w_dot)
        double ax_body __attribute__((unused)) = cosY*cosP*ax_ned + sinY*cosP*ay_ned - sinP*az_ned;
        double ay_body __attribute__((unused)) = (-sinY*cosR + cosY*sinP*sinR)*ax_ned + (cosY*cosR + sinY*sinP*sinR)*ay_ned + cosP*sinR*az_ned;
        double az_body = (sinY*sinR + cosY*sinP*cosR)*ax_ned + (-cosY*sinR + sinY*sinP*cosR)*ay_ned + cosP*cosR*az_ned;
        
        // G-force = body Z-axis acceleration / gravity (including gravity component)
        // Add gravity component: in body frame, gravity contributes 9.81 * cos(pitch) * cos(roll) to Z
        double gravityZ_body = 9.81 * cosP * cosR;
        gForce = (float)((az_body + gravityZ_body) / 9.81);
        
        lastVx = engineState.Vx;
        lastVy = engineState.Vy; 
        lastVz = engineState.Vz;
        lastTime = currentTime;
    }
    
    DrawRectangle(gX, gY, gWidth, gHeight, Fade(BLACK, 0.85f));
    DrawRectangleLines(gX, gY, gWidth, gHeight, WHITE);
    
    // G-force scale marks (-3G to +6G typical aircraft range)
    for (float g = -2.0f; g <= 5.0f; g += 1.0f) {
        if (g == 0) continue;
        float xPos = (gX + gWidth/2) + (g / 7.0f) * (gWidth * 0.8f);
        if (xPos >= gX + 5 && xPos <= gX + gWidth - 5) {
            DrawLine((int)xPos, gY + gHeight - 8, (int)xPos, gY + gHeight - 3, WHITE);
            sprintf(instText, "%.0f", g);
            int textW = MeasureText(instText, 6);
            DrawText(instText, (int)(xPos - textW/2), gY + 3, 6, WHITE);
        }
    }
    
    // Current G-force pointer
    float gClamped = fmin(fmax(-gForce, -3.0f), 6.0f);
    float gPointerX = (gX + gWidth/2) + (gClamped / 7.0f) * (gWidth * 0.8f);
    DrawTriangle({gPointerX, (float)(gY + gHeight - 3)}, 
                {gPointerX - 3.0f, (float)(gY + gHeight + 3)},
                {gPointerX + 3.0f, (float)(gY + gHeight + 3)}, 
                (-gForce > 4.0f || -gForce < -1.0f) ? RED : GREEN);
    
    // G-force digital readout
    sprintf(instText, "%.1fG", -gForce);
    Color gColor = (-gForce > 4.0f || -gForce < -1.0f) ? RED : 
                   (-gForce > 2.5f || -gForce < 0.5f) ? YELLOW : GREEN;
    int gTextWidth = MeasureText(instText, 8);
    DrawText(instText, gX + gWidth/2 - gTextWidth/2, gY + gHeight/2 - 4, 8, gColor);
    DrawText("G-FORCE", gX + 15, gY + gHeight + 5, 8, WHITE);
    
    // Instrument label
    DrawText("ATTITUDE + FPM", centerX - 45, centerY + radius + 10, 12, WHITE);
    
    // Current attitude values (digital display)
    char attText[64];
    sprintf(attText, "R:%+.0f° P:%+.0f°", rollDeg, pitchDeg);
    
    // Color code for extreme attitudes
    Color attTextColor = WHITE;
    if (fabsf(pitchDeg) > 75.0f) {
        attTextColor = RED; // Extreme pitch warning
    } else if (fabsf(rollDeg) > 60.0f) {
        attTextColor = YELLOW; // High bank angle warning
    }
    
    DrawText(attText, centerX - 45, centerY + radius + 25, 10, attTextColor);
    
    // Flight path information
    char fpText[64];
    double totalSpeed = sqrt(engineState.Vx * engineState.Vx + engineState.Vy * engineState.Vy + engineState.Vz * engineState.Vz);
    sprintf(fpText, "GS: %.1f m/s", totalSpeed);
    DrawText(fpText, centerX - 45, centerY + radius + 40, 8, GREEN);
    
    // Show AoA and sideslip digitally
    sprintf(fpText, "AoA: %+.1f° β: %+.1f°", currentAoA * 180.0 / M_PI, currentBeta * 180.0 / M_PI);
    DrawText(fpText, centerX - 60, centerY + radius + 55, 8, WHITE);
    
    // Extreme attitude warning
    if (fabsf(pitchDeg) > 80.0f) {
        DrawText("EXTREME PITCH", centerX - 50, centerY + radius + 70, 10, RED);
    }
}

void FlightSimulator::updateControls()
{
    // Keyboard flight controls
    // Aileron (Roll control) - A/D keys (A=left stick/left roll, D=right stick/right roll)
    if (IsKeyDown(KEY_A)) {
        controls.aileron = fmaxf(controls.aileron - 2.0f * GetFrameTime(), -1.0f);  // Stick left = Roll left
    } else if (IsKeyDown(KEY_D)) {
        controls.aileron = fminf(controls.aileron + 2.0f * GetFrameTime(), 1.0f);   // Stick right = Roll right
    } else {
        // Return to center when no input
        if (controls.aileron > 0.01f) {
            controls.aileron = fmaxf(controls.aileron - 4.0f * GetFrameTime(), 0.0f);
        } else if (controls.aileron < -0.01f) {
            controls.aileron = fminf(controls.aileron + 4.0f * GetFrameTime(), 0.0f);
        } else {
            controls.aileron = 0.0f;
        }
    }
    
    // Elevator (Pitch control) - W/S keys (INVERTED: W=down, S=up)
    if (IsKeyDown(KEY_W)) {
        controls.elevator = fmaxf(controls.elevator - 2.0f * GetFrameTime(), -1.0f);  // Nose down
    } else if (IsKeyDown(KEY_S)) {
        controls.elevator = fminf(controls.elevator + 2.0f * GetFrameTime(), 1.0f);   // Nose up
    } else {
        // Return to center when no input
        if (controls.elevator > 0.01f) {
            controls.elevator = fmaxf(controls.elevator - 4.0f * GetFrameTime(), 0.0f);
        } else if (controls.elevator < -0.01f) {
            controls.elevator = fminf(controls.elevator + 4.0f * GetFrameTime(), 0.0f);
        } else {
            controls.elevator = 0.0f;
        }
    }
    
    // Rudder (Yaw control) - Q/E keys
    if (IsKeyDown(KEY_Q)) {
        controls.rudder = fmaxf(controls.rudder - 2.0f * GetFrameTime(), -1.0f);  // Nose left
    } else if (IsKeyDown(KEY_E)) {
        controls.rudder = fminf(controls.rudder + 2.0f * GetFrameTime(), 1.0f);   // Nose right
    } else {
        // Return to center when no input
        if (controls.rudder > 0.01f) {
            controls.rudder = fmaxf(controls.rudder - 4.0f * GetFrameTime(), 0.0f);
        } else if (controls.rudder < -0.01f) {
            controls.rudder = fminf(controls.rudder + 4.0f * GetFrameTime(), 0.0f);
        } else {
            controls.rudder = 0.0f;
        }
    }
    
    // Throttle control - Shift/Ctrl keys
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        controls.throttle = fminf(controls.throttle + 1.0f * GetFrameTime(), 1.0f);  // Increase throttle
    } else if (IsKeyDown(KEY_LEFT_CONTROL)) {
        controls.throttle = fmaxf(controls.throttle - 1.0f * GetFrameTime(), 0.0f);  // Decrease throttle
    }

    // Joystick support: automatically pick the first available gamepad
    int detectedGamepad = -1;
    const int maxGamepadSlots = 4;
    for (int idx = 0; idx < maxGamepadSlots; ++idx) {
        if (IsGamepadAvailable(idx)) {
            detectedGamepad = idx;
            break;
        }
    }

    if (detectedGamepad != activeGamepadIndex) {
        activeGamepadIndex = detectedGamepad;
        if (activeGamepadIndex >= 0) {
            const char* name = GetGamepadName(activeGamepadIndex);
            activeGamepadName = name ? name : "Unknown";
            std::cout << "Gamepad connected on slot " << activeGamepadIndex << ": "
                      << activeGamepadName << std::endl;
        } else {
            if (!activeGamepadName.empty()) {
                std::cout << "Gamepad disconnected" << std::endl;
            }
            activeGamepadName.clear();
        }
    }

    if (activeGamepadIndex >= 0) {
        const float deadzone = 0.12f;

        auto applyDeadzone = [deadzone](float value) {
            if (fabsf(value) < deadzone) return 0.0f;
            float sign = (value > 0.0f) ? 1.0f : -1.0f;
            float magnitude = (fabsf(value) - deadzone) / (1.0f - deadzone);
            if (magnitude < 0.0f) magnitude = 0.0f;
            if (magnitude > 1.0f) magnitude = 1.0f;
            return magnitude * sign;
        };

        auto clampInput = [](float value) {
            if (value > 1.0f) return 1.0f;
            if (value < -1.0f) return -1.0f;
            return value;
        };

        int axisCount = GetGamepadAxisCount(activeGamepadIndex);
        lastGamepadAxisCount = axisCount;
        lastGamepadAxes.fill(0.0f);
        int sampleAxes = std::min<int>((int)lastGamepadAxes.size(), axisCount);
        for (int a = 0; a < sampleAxes; ++a) {
            lastGamepadAxes[a] = GetGamepadAxisMovement(activeGamepadIndex, a);
        }

        if (axisCount > GAMEPAD_AXIS_LEFT_X) {
            float rawRoll = GetGamepadAxisMovement(activeGamepadIndex, GAMEPAD_AXIS_LEFT_X);
            float rollInput = applyDeadzone(rawRoll);  // Invert X axis for correct roll direction
            lastGamepadRoll = rollInput;
            controls.aileron = clampInput(rollInput);
        } else {
            lastGamepadRoll = 0.0f;
        }

        if (axisCount > GAMEPAD_AXIS_LEFT_Y) {
            float rawPitch = GetGamepadAxisMovement(activeGamepadIndex, GAMEPAD_AXIS_LEFT_Y);
            float pitchInput = applyDeadzone(rawPitch);  // Use positive Y for pitch up
            lastGamepadPitch = pitchInput;
            // Positive Y should pitch nose up -> positive elevator
            controls.elevator = clampInput(pitchInput);
        } else {
            lastGamepadPitch = 0.0f;
        }
    } else {
        lastGamepadRoll = 0.0f;
        lastGamepadPitch = 0.0f;
        lastGamepadAxisCount = 0;
        lastGamepadAxes.fill(0.0f);
    }
}

void FlightSimulator::handleForceSample(const SimulationEngine::ForceVectorSample& sample)
{
    Vector3 position = { (float)sample.px, (float)sample.py, (float)sample.pz };
    Vector3 forceVec = { (float)sample.fx, (float)sample.fy, (float)sample.fz };

    Color forceColor = GREEN;
    std::string label = "Aero Force";

    if (sample.kind == SimulationEngine::ForceKind::Gravity) {
        forceColor = BLUE;
        label = "Gravity";
    } else {
        float magnitude = sqrtf(forceVec.x * forceVec.x + forceVec.y * forceVec.y + forceVec.z * forceVec.z);
        if (magnitude > 1000.0f) {
            forceColor = RED;
        } else if (magnitude > 500.0f) {
            forceColor = ORANGE;
        } else if (magnitude > 100.0f) {
            forceColor = YELLOW;
        } else {
            forceColor = GREEN;
        }
    }

    forceViz.addForce(position, forceVec, forceColor, label);
}

void FlightSimulator::handleMomentSample(const SimulationEngine::MomentVectorSample& sample)
{
    Vector3 position = { (float)sample.px, (float)sample.py, (float)sample.pz };
    Vector3 momentVec = { (float)sample.mx, (float)sample.my, (float)sample.mz };

    Color momentColor = PURPLE;
    std::string label = "Force Moment";

    if (sample.kind == SimulationEngine::MomentKind::Direct) {
        momentColor = MAGENTA;
        label = "Direct Moment";

        if (fabs(momentVec.x) > fabs(momentVec.y) && fabs(momentVec.x) > fabs(momentVec.z)) {
            momentColor = RED;
            label = "Roll Moment";
        } else if (fabs(momentVec.y) > fabs(momentVec.z)) {
            momentColor = GREEN;
            label = "Pitch Moment";
        } else {
            momentColor = BLUE;
            label = "Yaw Moment";
        }
    }

    forceViz.addMoment(position, momentVec, momentColor, label);
}

void FlightSimulator::drawForceVectors()
{
    if (!forceViz.showForces && !forceViz.showMoments) return;
    
    // Get aircraft position and orientation
    Vector3 aircraftPos = { (float)engineState.X, (float)(-engineState.Z), (float)engineState.Y };
    
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);
    
    // Draw force vectors
    if (forceViz.showForces) {
        for (const auto& force : forceViz.forces) {
            // Transform application point from body frame to world frame
            Vector3 worldPos;
            worldPos.x = aircraftPos.x + (float)(R[0][0] * force.position.x + R[0][1] * force.position.y + R[0][2] * force.position.z);
            worldPos.y = aircraftPos.y + (float)(-R[2][0] * force.position.x - R[2][1] * force.position.y - R[2][2] * force.position.z);
            worldPos.z = aircraftPos.z + (float)(R[1][0] * force.position.x + R[1][1] * force.position.y + R[1][2] * force.position.z);
            
            // Transform force vector from body frame to world frame
            Vector3 worldForce;
            worldForce.x = (float)(R[0][0] * force.force.x + R[0][1] * force.force.y + R[0][2] * force.force.z);
            worldForce.y = (float)(-R[2][0] * force.force.x - R[2][1] * force.force.y - R[2][2] * force.force.z);
            worldForce.z = (float)(R[1][0] * force.force.x + R[1][1] * force.force.y + R[1][2] * force.force.z);
            
            // Scale force vector for visualization
            Vector3 scaledForce = {
                worldForce.x * forceViz.forceScale,
                worldForce.y * forceViz.forceScale,
                worldForce.z * forceViz.forceScale
            };
            
            // Draw force vector as line
            Vector3 forceEnd = {
                worldPos.x + scaledForce.x,
                worldPos.y + scaledForce.y,
                worldPos.z + scaledForce.z
            };
            
            DrawLine3D(worldPos, forceEnd, force.color);
            
            // Draw application point
            DrawSphere(worldPos, 1.0f, force.color);
            
            // Draw arrowhead
            Vector3 forceDir = Vector3Normalize(scaledForce);
            float arrowSize = 3.0f;
            
            // Calculate perpendicular vectors for arrowhead
            Vector3 perp1 = Vector3CrossProduct(forceDir, {0, 1, 0});
            if (Vector3Length(perp1) < 0.1f) perp1 = Vector3CrossProduct(forceDir, {1, 0, 0});
            perp1 = Vector3Normalize(perp1);
            Vector3 perp2 = Vector3CrossProduct(forceDir, perp1);
            perp2 = Vector3Normalize(perp2);
            
            Vector3 arrowBase = {
                forceEnd.x - forceDir.x * arrowSize,
                forceEnd.y - forceDir.y * arrowSize,
                forceEnd.z - forceDir.z * arrowSize
            };
            
            Vector3 arrow1 = {
                arrowBase.x + perp1.x * arrowSize * 0.5f,
                arrowBase.y + perp1.y * arrowSize * 0.5f,
                arrowBase.z + perp1.z * arrowSize * 0.5f
            };
            
            Vector3 arrow2 = {
                arrowBase.x - perp1.x * arrowSize * 0.5f,
                arrowBase.y - perp1.y * arrowSize * 0.5f,
                arrowBase.z - perp1.z * arrowSize * 0.5f
            };
            
            Vector3 arrow3 = {
                arrowBase.x + perp2.x * arrowSize * 0.5f,
                arrowBase.y + perp2.y * arrowSize * 0.5f,
                arrowBase.z + perp2.z * arrowSize * 0.5f
            };
            
            Vector3 arrow4 = {
                arrowBase.x - perp2.x * arrowSize * 0.5f,
                arrowBase.y - perp2.y * arrowSize * 0.5f,
                arrowBase.z - perp2.z * arrowSize * 0.5f
            };
            
            DrawLine3D(forceEnd, arrow1, force.color);
            DrawLine3D(forceEnd, arrow2, force.color);
            DrawLine3D(forceEnd, arrow3, force.color);
            DrawLine3D(forceEnd, arrow4, force.color);
            
        }
    }
    
    // Draw moment vectors
    if (forceViz.showMoments) {
        for (const auto& moment : forceViz.moments) {
            // Transform application point from body frame to world frame
            Vector3 worldPos;
            worldPos.x = aircraftPos.x + (float)(R[0][0] * moment.position.x + R[0][1] * moment.position.y + R[0][2] * moment.position.z);
            worldPos.y = aircraftPos.y + (float)(-R[2][0] * moment.position.x - R[2][1] * moment.position.y - R[2][2] * moment.position.z);
            worldPos.z = aircraftPos.z + (float)(R[1][0] * moment.position.x + R[1][1] * moment.position.y + R[1][2] * moment.position.z);
            
            // Transform moment vector from body frame to world frame
            Vector3 worldMoment;
            worldMoment.x = (float)(R[0][0] * moment.force.x + R[0][1] * moment.force.y + R[0][2] * moment.force.z);
            worldMoment.y = (float)(-R[2][0] * moment.force.x - R[2][1] * moment.force.y - R[2][2] * moment.force.z);
            worldMoment.z = (float)(R[1][0] * moment.force.x + R[1][1] * moment.force.y + R[1][2] * moment.force.z);
            
            // Scale moment vector for visualization
            Vector3 scaledMoment = {
                worldMoment.x * forceViz.momentScale,
                worldMoment.y * forceViz.momentScale,
                worldMoment.z * forceViz.momentScale
            };
            
            // Draw moment as curved arrow (simplified as straight line with different thickness)
            Vector3 momentEnd = {
                worldPos.x + scaledMoment.x,
                worldPos.y + scaledMoment.y,
                worldPos.z + scaledMoment.z
            };
            
            // Draw thicker line for moments
            DrawLine3D(worldPos, momentEnd, moment.color);
            
            // Draw rotation indicator (circle around the moment axis)
            Vector3 momentDir = Vector3Normalize(scaledMoment);
            float circleRadius = 5.0f;
            
            // Create a circle perpendicular to the moment axis
            Vector3 perp1 = Vector3CrossProduct(momentDir, {0, 1, 0});
            if (Vector3Length(perp1) < 0.1f) perp1 = Vector3CrossProduct(momentDir, {1, 0, 0});
            perp1 = Vector3Normalize(perp1);
            Vector3 perp2 = Vector3CrossProduct(momentDir, perp1);
            perp2 = Vector3Normalize(perp2);
            
            const int circleSegments = 16;
            for (int i = 0; i < circleSegments; i++) {
                float angle1 = (float)(i * 2 * M_PI / circleSegments);
                float angle2 = (float)((i + 1) * 2 * M_PI / circleSegments);
                
                Vector3 p1 = {
                    worldPos.x + cosf(angle1) * perp1.x * circleRadius + sinf(angle1) * perp2.x * circleRadius,
                    worldPos.y + cosf(angle1) * perp1.y * circleRadius + sinf(angle1) * perp2.y * circleRadius,
                    worldPos.z + cosf(angle1) * perp1.z * circleRadius + sinf(angle1) * perp2.z * circleRadius
                };
                
                Vector3 p2 = {
                    worldPos.x + cosf(angle2) * perp1.x * circleRadius + sinf(angle2) * perp2.x * circleRadius,
                    worldPos.y + cosf(angle2) * perp1.y * circleRadius + sinf(angle2) * perp2.y * circleRadius,
                    worldPos.z + cosf(angle2) * perp1.z * circleRadius + sinf(angle2) * perp2.z * circleRadius
                };
                
                DrawLine3D(p1, p2, moment.color);
            }
            
            // Draw center point
            DrawSphere(worldPos, 1.5f, moment.color);
        }
    }
}
