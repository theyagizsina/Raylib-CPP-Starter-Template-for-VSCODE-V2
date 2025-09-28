#include "Application.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef SLUGFT2_TO_KGM2
#define SLUGFT2_TO_KGM2 1.355817962
#endif






Application::Application()
    : simulationEngine(engineState, bodyState, ofmInterface, &dataLogger)
{
}

Application::~Application()
{
}

bool Application::initialize()
{
    if (!SimulationConfig::getUserInput(engineState)) {
        ConsoleOutput::displayError("Failed to get user input");
        return false;
    }

    if (!ofmInterface.initialize()) {
        ConsoleOutput::displayError("Failed to initialize OFM interface");
        return false;
    }

    if (!dataLogger.initialize("flight_log.csv")) {
        ConsoleOutput::displayError("Failed to initialize data logger");
        return false;
    }

    simulationEngine.setDataLogger(&dataLogger);

    ofmInterface.init();
    ofmInterface.setMassState(
        engineState.mass,
        engineState.cmX, engineState.cmY, engineState.cmZ,
        engineState.moiX, engineState.moiY, engineState.moiZ
    );

    simulationEngine.initializeBodyState();
    simulationEngine.resetTime();

    return true;
}

int Application::run()
{
    double endTime = 120.0;
    double simTime = 0.0;
    auto prev = std::chrono::steady_clock::now();

    while (simTime < endTime)
    {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count();
        if (ms < (int)(engineState.dt * 1000)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        prev = now;

        simulationEngine.step(engineState.dt, controls);
        simTime = simulationEngine.getSimulationTime();

        if (engineState.Z >= 0.0) {
            ConsoleOutput::displayInfo("Ground contact. Breaking...");
            break;
        }
    }

    ConsoleOutput::displayInfo("Simulation ended.");
    return 0;
}

// Main function removed - using FlightSimulator instead
