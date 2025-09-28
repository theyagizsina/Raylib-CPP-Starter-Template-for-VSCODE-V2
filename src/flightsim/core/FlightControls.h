#pragma once

struct FlightControls {
    float aileron = 0.0f;   // -1.0 to 1.0 (roll)
    float elevator = 0.0f;  // -1.0 to 1.0 (pitch)
    float rudder = 0.0f;    // -1.0 to 1.0 (yaw)
    float throttle = 0.0f;  // 0.0 to 1.0
};
