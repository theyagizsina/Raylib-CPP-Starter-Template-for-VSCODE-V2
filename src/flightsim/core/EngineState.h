#pragma once
#include "../physics/Quaternion.h"

struct EngineState {
    // N,E,D
    double X, Y, Z;
    // NED hizlari
    double Vx, Vy, Vz;

    // oryantasyon quaternion
    FlightQuaternion q;

    // acisal hizlar (body)
    double p, q_, r;

    // k�tle, cm, moi
    double mass;
    double cmX, cmY, cmZ;
    double moiX, moiY, moiZ;

    // r�zgar NED
    double windX, windY, windZ;

    // dt
    double dt;

    // entegre icin
    int integratorType; // 0:euler,1:rk4
    int subSteps;       // sub stepping

    EngineState();
};