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

    ofmInterface.init();
    ofmInterface.setMassState(
        engineState.mass,
        engineState.cmX, engineState.cmY, engineState.cmZ,
        engineState.moiX, engineState.moiY, engineState.moiZ
    );

    initializeBodyState();

    return true;
}

void Application::initializeBodyState()
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

        simulationStep(simTime);
        simTime += engineState.dt;

        if (engineState.Z >= 0.0) {
            ConsoleOutput::displayInfo("Ground contact. Breaking...");
            break;
        }
    }

    ConsoleOutput::displayInfo("Simulation ended.");
    return 0;
}

void Application::simulationStep(double simTime)
{
    double altitude = -engineState.Z;
    double T, a, rho, pp;
    Atmosphere::get1976StandardAtmosphere(altitude, T, a, rho, pp);

    ofmInterface.setAtmosphere(altitude, T, a, rho, pp,
                              engineState.windX, engineState.windY, engineState.windZ);

    double subDt = engineState.dt / double(engineState.subSteps);
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

    double rollRad, pitchRad, yawRad;
    QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);

    double speedBody = std::sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
    double alphaDeg = 0.0, betaDeg = 0.0;
    if (speedBody > 1e-6) {
        alphaDeg = std::atan2(bodyState.w, bodyState.u);
        betaDeg = std::asin(bodyState.v / speedBody);
    }

    dataLogger.logFlightData(simTime, engineState.X, engineState.Y, engineState.Z,
                           rollRad, pitchRad, yawRad, alphaDeg, betaDeg);

    ConsoleOutput::displayFlightData(simTime, engineState.X, engineState.Y, engineState.Z,
                                   rollRad, pitchRad, yawRad);
}

void Application::calculateAeroForces(double& FxB, double& FyB, double& FzB,
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

void Application::addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB)
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

void Application::updatePhysics(double subDt, double FxB, double FyB, double FzB,
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

void Application::updatePosition(double subDt)
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

// Main function removed - using FlightSimulator instead
