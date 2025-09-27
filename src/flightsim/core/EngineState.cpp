#include "EngineState.h"

EngineState::EngineState()
{
    X = 0; Y = 0; Z = 0; Vx = 0; Vy = 0; Vz = 0;
    q.w = 1; q.x = 0; q.y = 0; q.z = 0;
    p = 0; q_ = 0; r = 0;
    mass = 85;
    cmX = 0; cmY = 0; cmZ = 0;
    moiX = 1000; moiY = 1000; moiZ = 1000;
    windX = 0; windY = 0; windZ = 0;
    dt = 0.01;
    integratorType = 0;
    subSteps = 1;
}