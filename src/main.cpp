#include <raylib.h>
#include "FlightSimulator.h"

int main()
{
    constexpr int screenWidth = 1200;
    constexpr int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "OpenFlightLab - Raylib Flight Simulator");
    SetTargetFPS(60);

    FlightSimulator simulator;

    if (!simulator.initialize()) {
        CloseWindow();
        return -1;
    }

    while (!WindowShouldClose())
    {
        simulator.update();
        simulator.draw();
    }

    CloseWindow();
    return 0;
}