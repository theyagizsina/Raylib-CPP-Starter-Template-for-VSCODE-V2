#include "FlightSimulator.h"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

FlightSimulator::FlightSimulator() : isSimulationRunning(false), simulationTime(0.0),
    cameraMode(FLIGHT_CAMERA_EXTERNAL), cameraDistance(150.0f), cameraYaw(0.0f), cameraPitch(-20.0f),
    currentAoA(0.0), currentBeta(0.0), currentAirspeed(0.0), currentClimbRate(0.0), lastAltitude(0.0)
{
    mouseLastPos = { 0, 0 };
    
    // Initialize flight controls
    controls.aileron = 0.0f;
    controls.elevator = 0.0f;
    controls.rudder = 0.0f;
    controls.throttle = 0.5f;  // Start with 50% throttle
}

FlightSimulator::~FlightSimulator()
{
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

    isSimulationRunning = true;

    return true;
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
    ClearBackground(SKYBLUE);

    BeginMode3D(camera);

    // Draw ground plane
    DrawPlane((Vector3){ 0, 0, 0 }, (Vector2){ 2000, 2000 }, GREEN);

    // Draw grid
    DrawGrid(100, 100.0f);

    // Draw aircraft
    drawAircraft();

    // Draw flight path
    drawFlightPath();

    EndMode3D();

    // Draw HUD
    drawHUD();

    EndDrawing();
}

void FlightSimulator::drawAircraft()
{
    Vector3 pos = { (float)engineState.X, (float)(-engineState.Z), (float)engineState.Y };

    // Get aircraft rotation matrix to determine orientation vectors
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);
    
    // Calculate forward, right, and up vectors from rotation matrix
    Vector3 forward = { (float)R[0][0], (float)(-R[2][0]), (float)R[1][0] };
    Vector3 right = { (float)R[0][1], (float)(-R[2][1]), (float)R[1][1] };
    Vector3 up = { (float)R[0][2], (float)(-R[2][2]), (float)R[1][2] };
    
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

void FlightSimulator::drawFlightPath()
{
    if (flightPath.size() < 2) return;

    for (size_t i = 1; i < flightPath.size(); i++) {
        DrawLine3D(flightPath[i-1], flightPath[i], BLUE);
    }
}

void FlightSimulator::drawHUD()
{
    // Enhanced flight data display
    double rollRad, pitchRad, yawRad;
    QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);

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
    
    // Attitude data
    sprintf(text, "Roll: %.1f°", rollRad * 180.0 / M_PI);
    DrawText(text, 10, 160, 18, WHITE);

    sprintf(text, "Pitch: %.1f°", pitchRad * 180.0 / M_PI);
    DrawText(text, 10, 180, 18, WHITE);

    sprintf(text, "Yaw: %.1f°", yawRad * 180.0 / M_PI);
    DrawText(text, 10, 200, 18, WHITE);

    // Controls
    DrawText("FLIGHT CONTROLS:", 10, 240, 14, YELLOW);
    DrawText("W/S - Elevator (W=Down, S=Up)", 10, 260, 12, WHITE);
    DrawText("A/D - Aileron (A=Left, D=Right)", 10, 275, 12, WHITE);
    DrawText("Q/E - Rudder (Yaw)", 10, 290, 12, WHITE);
    DrawText("Shift/Ctrl - Throttle", 10, 305, 12, WHITE);
    
    DrawText("CAMERA CONTROLS:", 10, 330, 14, YELLOW);
    DrawText("SPACE - Pause/Resume", 10, 350, 12, WHITE);
    DrawText("R - Reset Simulation", 10, 365, 12, WHITE);
    DrawText("C - Switch Camera", 10, 380, 12, WHITE);
    DrawText("Left Click+Drag - Camera", 10, 395, 12, WHITE);
    DrawText("Mouse Wheel - Zoom", 10, 410, 12, WHITE);
    
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
        camera.up = (Vector3){ (float)R[0][2], (float)(-R[2][2]), (float)R[1][2] };
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
    
    // Draw attitude indicator background
    DrawCircle(centerX, centerY, radius, Fade(BLACK, 0.7f));
    DrawCircleLines(centerX, centerY, radius, WHITE);
    
    // Draw horizon line (simplified)
    float pitchOffset = (float)(pitchRad * 100.0);
    float rollAngle = (float)rollRad;
    
    Vector2 p1 = { centerX - radius * cosf(rollAngle), centerY - radius * sinf(rollAngle) + pitchOffset };
    Vector2 p2 = { centerX + radius * cosf(rollAngle), centerY + radius * sinf(rollAngle) + pitchOffset };
    
    // Sky (above horizon)
    if (pitchOffset < radius) {
        DrawCircle(centerX, centerY - (int)pitchOffset, radius, Fade(SKYBLUE, 0.5f));
    }
    
    // Ground (below horizon)
    if (pitchOffset > -radius) {
        DrawCircle(centerX, centerY - (int)pitchOffset + radius, radius, Fade(BROWN, 0.5f));
    }
    
    // Horizon line
    DrawLineEx(p1, p2, 3.0f, WHITE);
    
    // Aircraft symbol (fixed in center)
    DrawLine(centerX - 20, centerY, centerX + 20, centerY, YELLOW);
    DrawLine(centerX - 15, centerY - 5, centerX - 15, centerY + 5, YELLOW);
    DrawLine(centerX + 15, centerY - 5, centerX + 15, centerY + 5, YELLOW);
    DrawCircle(centerX, centerY, 3, YELLOW);
    
    // Roll scale marks
    for (int i = -60; i <= 60; i += 30) {
        if (i == 0) continue;
        float angle = (float)(i * M_PI / 180.0);
        Vector2 start = { centerX + (radius - 10) * sinf(angle), centerY - (radius - 10) * cosf(angle) };
        Vector2 end = { centerX + radius * sinf(angle), centerY - radius * cosf(angle) };
        DrawLineEx(start, end, 2.0f, WHITE);
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
