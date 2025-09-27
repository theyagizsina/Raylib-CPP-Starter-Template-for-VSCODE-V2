#include "ConsoleOutput.h"
#include <iostream>

void ConsoleOutput::displayFlightData(double simTime, double X, double Y, double Z,
                                     double rollDeg, double pitchDeg, double yawDeg)
{
    std::cout << "t=" << simTime << " posNED=(" << X << "," << Y << "," << Z << ") "
              << " roll/pitch/yaw=" << rollDeg << "," << pitchDeg << "," << yawDeg
              << "\n";
}

void ConsoleOutput::displayInfo(const char* message)
{
    std::cout << "[INFO] " << message << "\n";
}

void ConsoleOutput::displayError(const char* message)
{
    std::cerr << "[ERROR] " << message << "\n";
}