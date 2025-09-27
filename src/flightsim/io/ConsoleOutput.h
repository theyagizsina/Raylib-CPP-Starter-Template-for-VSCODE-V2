#pragma once

class ConsoleOutput {
public:
    static void displayFlightData(double simTime, double X, double Y, double Z,
                                 double rollDeg, double pitchDeg, double yawDeg);
    static void displayInfo(const char* message);
    static void displayError(const char* message);
};