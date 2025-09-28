#include "SimulationEngine.h"

#include <cmath>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SimulationEngine::SimulationEngine(EngineState& engineStateRef,
                                   State6& bodyStateRef,
                                   OFMInterface& ofmInterfaceRef,
                                   DataLogger* dataLoggerPtr)
    : engineState(engineStateRef),
      bodyState(bodyStateRef),
      ofmInterface(ofmInterfaceRef),
      dataLogger(dataLoggerPtr),
      simulationTime(0.0),
      logCounter(0) {}

void SimulationEngine::setForceCallback(ForceCallback forceCb, MomentCallback momentCb)
{
    forceCallback = std::move(forceCb);
    momentCallback = std::move(momentCb);
}

void SimulationEngine::setDataLogger(DataLogger* logger)
{
    dataLogger = logger;
}

void SimulationEngine::initializeBodyState()
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

void SimulationEngine::resetTime(double timeSeconds)
{
    simulationTime = timeSeconds;
    logCounter = 0;
}

double SimulationEngine::getSimulationTime() const
{
    return simulationTime;
}

void SimulationEngine::step(double deltaTime, const FlightControls& controls)
{
    if (engineState.subSteps <= 0) {
        engineState.subSteps = 1;
    }

    double altitude = -engineState.Z;
    double T = 0.0;
    double a = 0.0;
    double rho = 0.0;
    double pp = 0.0;
    Atmosphere::get1976StandardAtmosphere(altitude, T, a, rho, pp);

    ofmInterface.setAtmosphere(altitude, T, a, rho, pp,
                               engineState.windX, engineState.windY, engineState.windZ);

    sendControlsToOFM(controls);

    double subDt = deltaTime / static_cast<double>(engineState.subSteps);

    for (int si = 0; si < engineState.subSteps; ++si) {
        double speedBody = std::sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
        double alpha = 0.0;
        double beta = 0.0;
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

        double FxB = 0.0;
        double FyB = 0.0;
        double FzB = 0.0;
        double MxB = 0.0;
        double MyB = 0.0;
        double MzB = 0.0;

        calculateAeroForces(FxB, FyB, FzB, MxB, MyB, MzB);

        double R[3][3];
        QuaternionOperations::toMatrix(engineState.q, R);
        addGravitationalForces(R, FxB, FyB, FzB);

        updatePhysics(subDt, FxB, FyB, FzB, MxB, MyB, MzB);
        updatePosition(subDt);
    }

    simulationTime += deltaTime;
    logFlightData();
}

void SimulationEngine::sendControlsToOFM(const FlightControls& controls)
{
    ofmInterface.setCommand(0, controls.aileron);
    ofmInterface.setCommand(1, controls.elevator);
    ofmInterface.setCommand(2, controls.rudder);
    ofmInterface.setCommand(3, controls.throttle);
}

void SimulationEngine::calculateAeroForces(double& FxB, double& FyB, double& FzB,
                                           double& MxB, double& MyB, double& MzB)
{
    if (forceCallback) {
        ForceVectorSample gravitySample{};
        gravitySample.px = 0.0;
        gravitySample.py = 0.0;
        gravitySample.pz = 0.0;
        gravitySample.fx = 0.0;
        gravitySample.fy = engineState.mass * 9.81;
        gravitySample.fz = 0.0;
        gravitySample.kind = ForceKind::Gravity;
        forceCallback(gravitySample);
    }

    while (true) {
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        double px = 0.0;
        double py = 0.0;
        double pz = 0.0;
        bool ok = ofmInterface.addLocalForceComponent(fx, fy, fz, px, py, pz);
        if (!ok) {
            break;
        }
        FxB += fx;
        FyB += fy;
        FzB += fz;

        if (forceCallback) {
            ForceVectorSample sample{};
            sample.px = px;
            sample.py = py;
            sample.pz = pz;
            sample.fx = fx;
            sample.fy = fy;
            sample.fz = fz;
            sample.kind = ForceKind::Aerodynamic;
            forceCallback(sample);
        }

        double rx = px - engineState.cmX;
        double ry = py - engineState.cmY;
        double rz = pz - engineState.cmZ;
        double Mx_ = ry * fz - rz * fy;
        double My_ = rz * fx - rx * fz;
        double Mz_ = rx * fy - ry * fx;
        MxB += Mx_;
        MyB += My_;
        MzB += Mz_;

        if (momentCallback) {
            MomentVectorSample momentSample{};
            momentSample.px = engineState.cmX;
            momentSample.py = engineState.cmY;
            momentSample.pz = engineState.cmZ;
            momentSample.mx = Mx_;
            momentSample.my = My_;
            momentSample.mz = Mz_;
            momentSample.kind = MomentKind::Induced;
            momentCallback(momentSample);
        }
    }

    while (true) {
        double mx = 0.0;
        double my = 0.0;
        double mz = 0.0;
        bool ok = ofmInterface.addLocalMomentComponent(mx, my, mz);
        if (!ok) {
            break;
        }
        MxB += mx;
        MyB += my;
        MzB += mz;

        if (momentCallback) {
            MomentVectorSample momentSample{};
            momentSample.px = engineState.cmX;
            momentSample.py = engineState.cmY;
            momentSample.pz = engineState.cmZ;
            momentSample.mx = mx;
            momentSample.my = my;
            momentSample.mz = mz;
            momentSample.kind = MomentKind::Direct;
            momentCallback(momentSample);
        }
    }
}

void SimulationEngine::addGravitationalForces(const double R[3][3], double& FxB, double& FyB, double& FzB)
{
    double GxNED = 0.0;
    double GyNED = 0.0;
    double GzNED = engineState.mass * 9.81;

    double Rt[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
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

void SimulationEngine::updatePhysics(double subDt, double FxB, double FyB, double FzB,
                                     double MxB, double MyB, double MzB)
{
    if (engineState.integratorType == 0) {
        FlightDynamics::eulerIntegrate(bodyState, subDt, FxB, FyB, FzB, MxB, MyB, MzB,
                                       engineState.mass, engineState.moiX, engineState.moiY, engineState.moiZ);
    } else {
        FlightDynamics::rk4Integrate(bodyState, subDt, FxB, FyB, FzB, MxB, MyB, MzB,
                                     engineState.mass, engineState.moiX, engineState.moiY, engineState.moiZ);
    }

    QuaternionOperations::integrate(engineState.q, bodyState.p, bodyState.q, bodyState.r, subDt);
}

void SimulationEngine::updatePosition(double subDt)
{
    double R[3][3];
    QuaternionOperations::toMatrix(engineState.q, R);
    double vxN = 0.0;
    double vyE = 0.0;
    double vzD = 0.0;
    QuaternionOperations::bodyToWorld(R, bodyState.u, bodyState.v, bodyState.w, vxN, vyE, vzD);
    engineState.Vx = vxN;
    engineState.Vy = vyE;
    engineState.Vz = vzD;

    engineState.X += engineState.Vx * subDt;
    engineState.Y += engineState.Vy * subDt;
    engineState.Z += engineState.Vz * subDt;
}

void SimulationEngine::logFlightData()
{
    if (!dataLogger) {
        return;
    }

    ++logCounter;
    if (logCounter % 10 != 0) {
        return;
    }

    double rollRad = 0.0;
    double pitchRad = 0.0;
    double yawRad = 0.0;
    QuaternionOperations::toEulerZYX(engineState.q, rollRad, pitchRad, yawRad);

    double speedBody = std::sqrt(bodyState.u * bodyState.u + bodyState.v * bodyState.v + bodyState.w * bodyState.w);
    double alphaDeg = 0.0;
    double betaDeg = 0.0;
    if (speedBody > 1e-6) {
        alphaDeg = std::atan2(bodyState.w, bodyState.u) * 180.0 / M_PI;
        betaDeg = std::asin(bodyState.v / speedBody) * 180.0 / M_PI;
    }

    dataLogger->logFlightData(simulationTime,
                              engineState.X, engineState.Y, engineState.Z,
                              rollRad * 180.0 / M_PI,
                              pitchRad * 180.0 / M_PI,
                              yawRad * 180.0 / M_PI,
                              alphaDeg,
                              betaDeg);
}
