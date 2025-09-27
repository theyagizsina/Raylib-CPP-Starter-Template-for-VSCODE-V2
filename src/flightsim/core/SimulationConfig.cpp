#include "SimulationConfig.h"
#include "../physics/Quaternion.h"

bool SimulationConfig::getUserInput(EngineState& state)
{
    std::cout << "Initial N,E,D (m): ";
    std::cin >> state.X >> state.Y >> state.Z;

    std::cout << "Initial Vn,Ve,Vd (m/s): ";
    std::cin >> state.Vx >> state.Vy >> state.Vz;

    // orientation
    std::cout << "Enter orientation quaternion (w,x,y,z): ";
    std::cin >> state.q.w >> state.q.x >> state.q.y >> state.q.z;
    QuaternionOperations::normalize(state.q);

    std::cout << "Enter body rates p,q,r (rad/s): ";
    std::cin >> state.p >> state.q_ >> state.r;

    std::cout << "mass: ";
    std::cin >> state.mass;

    std::cout << "moiX,moiY,moiZ: ";
    std::cin >> state.moiX >> state.moiY >> state.moiZ;

    std::cout << "CofG X,Y,Z (body coords): ";
    std::cin >> state.cmX >> state.cmY >> state.cmZ;

    std::cout << "windN,windE,windD: ";
    std::cin >> state.windX >> state.windY >> state.windZ;

    std::cout << "dt (s): ";
    std::cin >> state.dt;

    std::cout << "Integrator (0=Euler,1=RK4): ";
    std::cin >> state.integratorType;

    std::cout << "Sub steps (int, e.g. 1,2,5): ";
    std::cin >> state.subSteps;

    return true;
}