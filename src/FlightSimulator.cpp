#include "FlightSimulator.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <raymath.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

FlightSimulator::FlightSimulator() : isSimulationRunning(false), simulationTime(0.0),
    cameraMode(FLIGHT_CAMERA_EXTERNAL), cameraDistance(150.0f), cameraYaw(0.0f), cameraPitch(-20.0f),
    activeGamepadIndex(-1), lastGamepadRoll(0.0f), lastGamepadPitch(0.0f),
    lastGamepadAxisCount(0), aircraftModelBaseTransform(MatrixIdentity()), customModelScale(1.0f), hasCustomModel(false),
    currentAoA(0.0), currentBeta(0.0), currentAirspeed(0.0), currentClimbRate(0.0), lastAltitude(0.0)
{
    mouseLastPos = { 0, 0 };
    lastGamepadAxes.fill(0.0f);
    
    // Initialize flight controls
    controls.aileron = 0.0f;
    controls.elevator = 0.0f;
    controls.rudder = 0.0f;
    controls.throttle = 0.5f;  // Start with 50% throttle
}

FlightSimulator::~FlightSimulator()
{
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

    ofmInterface.init();
    ofmInterface.setMassState(
        engineState.mass,
        engineState.cmX, engineState.cmY, engineState.cmZ,
        engineState.moiX, engineState.moiY, engineState.moiZ
    );

    initializeBodyState();

    // Setup 3D camera
    camera.position = (Vector3){ 100.0f, 50.0f, 100.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
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
    engineState.Vx = 200; engineState.Vy = 0; engineState.Vz = 0; // 50 m/s forward

    // Level flight attitude (no initial roll angle)
    engineState.q.w = 1; engineState.q.x = 0; engineState.q.y = 0; engineState.q.z = 0;

    // Initial angular velocities - roll moment for aileron deflection
    engineState.p = 0; // Roll angular velocity (rad/s) - aileron creates roll moment
    engineState.q_ = 0;  // No pitch moment
    engineState.r = 0;   // No yaw moment
    
    engineState.mass = 1000; // 1000 kg aircraft
    engineState.cmX = 0; engineState.cmY = 0; engineState.cmZ = 0;
    engineState.moiX = 5000; engineState.moiY = 8000; engineState.moiZ = 12000;
    engineState.windX = 0; engineState.windY = 10; engineState.windZ = 0;
    engineState.dt = 0.016; // 60 FPS
    engineState.integratorType = 1; // RK4
    engineState.subSteps = 1;
}

void FlightSimulator::initializeBodyState()
{
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);

    double vb_x = R[0][0] * engineState.Vx + R[1][0] * engineState.Vy + R[2][0] * engineState.Vz;
    double vb_y = R[0][1] * engineState.Vx + R[1][1] * engineState.Vy + R[2][1] * engineState.Vz;
    double vb_z = R[0][2] * engineState.Vx + R[1][2] * engineState.Vy + R[2][2] * engineState.Vz;

    bodyState.u = vb_x;
    bodyState.v = vb_y;
    bodyState.w = vb_z;
    bodyState.p = engineState.p;
    bodyState.q = engineState.q_;
    bodyState.r = engineState.r;
}

void FlightSimulator::update()
{
    if (!isSimulationRunning) return;

    handleInput();
    updateControls();
    updateCameraSystem();

    double deltaTime = GetFrameTime();
    simulationStep(deltaTime);
    simulationTime += deltaTime;
    
    updateFlightData();

    // Add current position to flight path
    flightPath.push_back((Vector3){ (float)engineState.X, (float)(-engineState.Z), (float)engineState.Y });

    // Limit flight path length
    if (flightPath.size() > 1000) {
        flightPath.erase(flightPath.begin());
    }

    // Stop simulation if aircraft hits ground
    if (engineState.Z >= 0.0) {
        isSimulationRunning = false;
        std::cout << "Ground contact. Simulation stopped." << std::endl;
    }
}

void FlightSimulator::simulationStep(double deltaTime)
{
    double altitude = -engineState.Z;
    double T, a, rho, pp;
    Atmosphere::get1976StandardAtmosphere(altitude, T, a, rho, pp);

    ofmInterface.setAtmosphere(altitude, T, a, rho, pp,
                              engineState.windX, engineState.windY, engineState.windZ);

    // Send current control inputs to OFM
    sendControlsToOFM();

    double subDt = deltaTime / double(engineState.subSteps);
    for (int si = 0; si < engineState.subSteps; si++) {
        double speedBody = std::sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
        double alpha = 0.0, beta = 0.0;
        if (speedBody > 1e-6) {
            alpha = std::atan2(bodyState.w, bodyState.u);
            beta = std::asin(bodyState.v / speedBody);
        }

        ofmInterface.setBodyState(
            0, 0, 0,
            bodyState.u, bodyState.v, bodyState.w,
            engineState.windX, engineState.windY, engineState.windZ,
            0, 0, 0,
            bodyState.p, bodyState.q, bodyState.r,
            0, 0, 0,
            alpha, beta
        );

        ofmInterface.simulate(subDt);

        double FxB = 0, FyB = 0, FzB = 0;
        double MxB = 0, MyB = 0, MzB = 0;
        calculateAeroForces(FxB, FyB, FzB, MxB, MyB, MzB);

        double R[3][3];
        QuaternionOperations::toMatrix(engineState.q, R);
        addGravitationalForces(R, FxB, FyB, FzB);

        updatePhysics(subDt, FxB, FyB, FzB, MxB, MyB, MzB);
        updatePosition(subDt);
    }

    // Log data occasionally
    static int logCounter = 0;
    if (++logCounter % 10 == 0) {
        double rollRad, pitchRad, yawRad;
        QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);

        double speedBody = std::sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
        double alphaDeg = 0.0, betaDeg = 0.0;
        if (speedBody > 1e-6) {
            alphaDeg = std::atan2(bodyState.w, bodyState.u) * 180.0 / M_PI;
            betaDeg = std::asin(bodyState.v / speedBody) * 180.0 / M_PI;
        }

        dataLogger.logFlightData(simulationTime, engineState.X, engineState.Y, engineState.Z,
                               rollRad * 180.0 / M_PI, pitchRad * 180.0 / M_PI, yawRad * 180.0 / M_PI,
                               alphaDeg, betaDeg);
    }
}

void FlightSimulator::calculateAeroForces(double& FxB, double& FyB, double& FzB,
                                        double& MxB, double& MyB, double& MzB)
{
    while (true) {
        double fx = 0, fy = 0, fz = 0, px = 0, py = 0, pz = 0;
        bool ok = ofmInterface.addLocalForceComponent(fx, fy, fz, px, py, pz);
        if (!ok) break;
        FxB += fx; FyB += fy; FzB += fz;

        double rx = px - engineState.cmX;
        double ry = py - engineState.cmY;
        double rz = pz - engineState.cmZ;
        double Mx_ = ry * fz - rz * fy;
        double My_ = rz * fx - rx * fz;
        double Mz_ = rx * fy - ry * fx;
        MxB += Mx_; MyB += My_; MzB += Mz_;
    }

    while (true) {
        double mx = 0, my = 0, mz = 0;
        bool ok = ofmInterface.addLocalMomentComponent(mx, my, mz);
        if (!ok) break;
        MxB += mx; MyB += my; MzB += mz;
    }
}

void FlightSimulator::addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB)
{
    double GxNED = 0, GyNED = 0, GzNED = engineState.mass * 9.81;

    double Rt[3][3];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            Rt[r][c] = R[c][r];
        }
    }

    double Gbx = Rt[0][0] * GxNED + Rt[0][1] * GyNED + Rt[0][2] * GzNED;
    double Gby = Rt[1][0] * GxNED + Rt[1][1] * GyNED + Rt[1][2] * GzNED;
    double Gbz = Rt[2][0] * GxNED + Rt[2][1] * GyNED + Rt[2][2] * GzNED;

    FxB += Gbx;
    FyB += Gby;
    FzB += Gbz;
}

void FlightSimulator::updatePhysics(double subDt, double FxB, double FyB, double FzB,
                                  double MxB, double MyB, double MzB)
{
    if (engineState.integratorType == 0)
        FlightDynamics::eulerIntegrate(bodyState, subDt, FxB, FyB, FzB, MxB, MyB, MzB,
                                     engineState.mass, engineState.moiX, engineState.moiY, engineState.moiZ);
    else
        FlightDynamics::rk4Integrate(bodyState, subDt, FxB, FyB, FzB, MxB, MyB, MzB,
                                   engineState.mass, engineState.moiX, engineState.moiY, engineState.moiZ);

    QuaternionOperations::integrate(engineState.q, bodyState.p, bodyState.q, bodyState.r, subDt);
}

void FlightSimulator::updatePosition(double subDt)
{
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);
    double vxN, vyE, vzD;
    QuaternionOperations::bodyToWorld(R, bodyState.u, bodyState.v, bodyState.w, vxN, vyE, vzD);
    engineState.Vx = vxN;
    engineState.Vy = vyE;
    engineState.Vz = vzD;

    engineState.X += engineState.Vx * subDt;
    engineState.Y += engineState.Vy * subDt;
    engineState.Z += engineState.Vz * subDt;
}

void FlightSimulator::handleInput()
{
    if (IsKeyPressed(KEY_SPACE)) {
        isSimulationRunning = !isSimulationRunning;
    }

    if (IsKeyPressed(KEY_R)) {
        // Reset simulation
        setDefaultConfiguration();
        initializeBodyState();
        flightPath.clear();
        simulationTime = 0.0;
        lastAltitude = -engineState.Z;
        isSimulationRunning = true;
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
    
    // Mouse wheel for distance
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
    
    sprintf(text, "Time: %.1fs", simulationTime);
    DrawText(text, 10, 30, 18, WHITE);

    sprintf(text, "Altitude: %.0fm", -engineState.Z);
    DrawText(text, 10, 50, 18, WHITE);

    sprintf(text, "Airspeed: %.1f m/s", currentAirspeed);
    DrawText(text, 10, 70, 18, WHITE);
    
    sprintf(text, "Climb Rate: %.1f m/s", currentClimbRate);
    DrawText(text, 10, 90, 18, currentClimbRate > 0 ? GREEN : (currentClimbRate < -5 ? RED : WHITE));

    sprintf(text, "AoA: %.2f°", currentAoA * 180.0 / M_PI);
    DrawText(text, 10, 110, 18, fabs(currentAoA) > 0.3 ? RED : WHITE);

    sprintf(text, "Sideslip: %.2f°", currentBeta * 180.0 / M_PI);
    DrawText(text, 10, 130, 18, fabs(currentBeta) > 0.1 ? YELLOW : WHITE);
    
    // Attitude data (Aviation Standard Format)
    DrawText("ATTITUDE:", 10, 150, 14, YELLOW);
    
    sprintf(text, "Roll: %+.1f°", rollDeg);
    Color rollColor = (fabs(rollDeg) > 45.0) ? RED : (fabs(rollDeg) > 20.0) ? YELLOW : WHITE;
    DrawText(text, 10, 170, 18, rollColor);

    sprintf(text, "Pitch: %+.1f°", pitchDeg);
    Color pitchColor = (fabs(pitchDeg) > 30.0) ? RED : (fabs(pitchDeg) > 15.0) ? YELLOW : WHITE;
    DrawText(text, 10, 190, 18, pitchColor);

    sprintf(text, "Heading: %03.0f°", headingDeg);
    DrawText(text, 10, 210, 18, WHITE);

    // Controls
    DrawText("FLIGHT CONTROLS:", 10, 250, 14, YELLOW);
    DrawText("W/S - Elevator (W=Down, S=Up)", 10, 270, 12, WHITE);
    DrawText("A/D - Aileron (A=Left, D=Right)", 10, 285, 12, WHITE);
    DrawText("Q/E - Rudder (Yaw)", 10, 300, 12, WHITE);
    DrawText("Shift/Ctrl - Throttle", 10, 315, 12, WHITE);
    
    DrawText("CAMERA CONTROLS:", 10, 340, 14, YELLOW);
    DrawText("SPACE - Pause/Resume", 10, 360, 12, WHITE);
    DrawText("R - Reset Simulation", 10, 375, 12, WHITE);
    DrawText("C - Switch Camera", 10, 390, 12, WHITE);
    DrawText("Left Click+Drag - Camera", 10, 405, 12, WHITE);
    DrawText("Mouse Wheel - Zoom", 10, 420, 12, WHITE);
    DrawText("L - Load Aircraft Model", 10, 435, 12, WHITE);
    DrawText("M - Toggle Model Display", 10, 450, 12, WHITE);
    
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
    
    // Camera mode indicator
    sprintf(text, "Camera: %s", cameraMode == FLIGHT_CAMERA_COCKPIT ? "COCKPIT" : "EXTERNAL");
    DrawText(text, GetScreenWidth() - 200, 10, 16, YELLOW);

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
    
    if (cameraMode == FLIGHT_CAMERA_COCKPIT) {
        // Cockpit view - camera inside aircraft with exact aircraft orientation
        double R[3][3];
        QuaternionOperations::toMatrix(engineState.q, R);
        
        // Position camera at aircraft location (slightly forward for better view)
        Vector3 cockpitOffset = { (float)(R[0][0] * 2.0), (float)(-R[2][0] * 2.0), (float)(R[1][0] * 2.0) };
        camera.position = (Vector3){ aircraftPos.x + cockpitOffset.x, 
                                     aircraftPos.y + cockpitOffset.y, 
                                     aircraftPos.z + cockpitOffset.z };
        
        // Forward direction in world coordinates (aircraft's nose direction)
    Vector3 forward = { (float)R[0][0], (float)(-R[2][0]), (float)R[1][0] };
        camera.target = (Vector3){ camera.position.x + forward.x * 100.0f, 
                                   camera.position.y + forward.y * 100.0f, 
                                   camera.position.z + forward.z * 100.0f };
        
        // Up vector in world coordinates (aircraft's up direction)
    camera.up = (Vector3){ (float)(-R[0][2]), (float)R[2][2], (float)(-R[1][2]) };
    } else {
        // External view with mouse control
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

void FlightSimulator::updateFlightData()
{
    // Calculate current airspeed
    currentAirspeed = sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
    
    // Calculate angle of attack and sideslip
    if (currentAirspeed > 1e-6) {
        currentAoA = atan2(bodyState.w, bodyState.u);
        currentBeta = asin(bodyState.v / currentAirspeed);
    } else {
        currentAoA = 0.0;
        currentBeta = 0.0;
    }
    
    // Calculate climb rate
    double currentAltitude = -engineState.Z;
    currentClimbRate = (currentAltitude - lastAltitude) / GetFrameTime();
    lastAltitude = currentAltitude;
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
                    sprintf(instText, alt >= 1000 ? "%.0fK" : "%d", alt >= 1000 ? alt/1000.0f : alt);
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
        double ax_body = cosY*cosP*ax_ned + sinY*cosP*ay_ned - sinP*az_ned;
        double ay_body = (-sinY*cosR + cosY*sinP*sinR)*ax_ned + (cosY*cosR + sinY*sinP*sinR)*ay_ned + cosP*sinR*az_ned;
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
    DrawText("ATTITUDE", centerX - 35, centerY + radius + 10, 12, WHITE);
    
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
    
    // Extreme attitude warning
    if (fabsf(pitchDeg) > 80.0f) {
        DrawText("EXTREME PITCH", centerX - 50, centerY + radius + 40, 10, RED);
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

void FlightSimulator::sendControlsToOFM()
{
    // Send control inputs to OFM
    // Command IDs may need to be adjusted based on actual OFM API
    ofmInterface.setCommand(0, controls.aileron);   // Aileron command
    ofmInterface.setCommand(1, controls.elevator);  // Elevator command  
    ofmInterface.setCommand(2, controls.rudder);    // Rudder command
    ofmInterface.setCommand(3, controls.throttle);  // Throttle command
}
