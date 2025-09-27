#include "OFMInterface.h"
#include <iostream>

OFMInterface::OFMInterface() : hModule(nullptr), fn_init(nullptr), fn_setCmd(nullptr),
    fn_setAtmos(nullptr), fn_setMass(nullptr), fn_setBodyState(nullptr),
    fn_simulate(nullptr), fn_addForce(nullptr), fn_addMoment(nullptr)
{
}

OFMInterface::~OFMInterface()
{
    shutdown();
}

bool OFMInterface::initialize()
{
#ifdef _WIN32
    hModule = LoadLibraryA("OpenFlightEngine.dll");
    if (!hModule) {
        std::cerr << "Cannot load OpenFlightEngine.dll\n";
        return false;
    }

    fn_init = (ofm_initialize_t)GetProcAddress(hModule, "ofm_initialize");
    fn_setCmd = (ofm_set_command_t)GetProcAddress(hModule, "ofm_set_command");
    fn_setAtmos = (ofm_set_atmosphere_t)GetProcAddress(hModule, "ofm_set_atmosphere");
    fn_setMass = (ofm_set_current_mass_state_t)GetProcAddress(hModule, "ofm_set_current_mass_state");
    fn_setBodyState = (ofm_set_current_state_body_axis_t)GetProcAddress(hModule, "ofm_set_current_state_body_axis");
    fn_simulate = (ofm_simulate_t)GetProcAddress(hModule, "ofm_simulate");
    fn_addForce = (ofm_add_local_force_component_t)GetProcAddress(hModule, "ofm_add_local_force_component");
    fn_addMoment = (ofm_add_local_moment_component_t)GetProcAddress(hModule, "ofm_add_local_moment_component");
#else
    hModule = dlopen("OpenFlightEngine.so", RTLD_LAZY);
    if (!hModule) {
        std::cerr << "Cannot load OpenFlightEngine.so: " << dlerror() << "\n";
        return false;
    }

    fn_init = (ofm_initialize_t)dlsym(hModule, "ofm_initialize");
    fn_setCmd = (ofm_set_command_t)dlsym(hModule, "ofm_set_command");
    fn_setAtmos = (ofm_set_atmosphere_t)dlsym(hModule, "ofm_set_atmosphere");
    fn_setMass = (ofm_set_current_mass_state_t)dlsym(hModule, "ofm_set_current_mass_state");
    fn_setBodyState = (ofm_set_current_state_body_axis_t)dlsym(hModule, "ofm_set_current_state_body_axis");
    fn_simulate = (ofm_simulate_t)dlsym(hModule, "ofm_simulate");
    fn_addForce = (ofm_add_local_force_component_t)dlsym(hModule, "ofm_add_local_force_component");
    fn_addMoment = (ofm_add_local_moment_component_t)dlsym(hModule, "ofm_add_local_moment_component");
#endif

    if (!fn_init || !fn_setCmd || !fn_setAtmos || !fn_setMass ||
        !fn_setBodyState || !fn_simulate || !fn_addForce || !fn_addMoment) {
        std::cerr << "Missing ofm function(s)\n";
        shutdown();
        return false;
    }

    return true;
}

void OFMInterface::shutdown()
{
    if (hModule) {
#ifdef _WIN32
        FreeLibrary(hModule);
#else
        dlclose(hModule);
#endif
        hModule = nullptr;
    }
}

void OFMInterface::init()
{
    if (fn_init) fn_init();
}

void OFMInterface::setCommand(int cmd, float value)
{
    if (fn_setCmd) fn_setCmd(cmd, value);
}

void OFMInterface::setAtmosphere(double altitude, double T, double a, double rho, double p,
                                double windX, double windY, double windZ)
{
    if (fn_setAtmos) fn_setAtmos(altitude, T, a, rho, p, windX, windY, windZ);
}

void OFMInterface::setMassState(double mass, double cmX, double cmY, double cmZ,
                               double moiX, double moiY, double moiZ)
{
    if (fn_setMass) fn_setMass(mass, cmX, cmY, cmZ, moiX, moiY, moiZ);
}

void OFMInterface::setBodyState(double ax, double ay, double az,
                               double vx, double vy, double vz,
                               double wind_x, double wind_y, double wind_z,
                               double dotx, double doty, double dotz,
                               double omegax, double omegay, double omegaz,
                               double yaw, double pitch, double roll,
                               double alpha, double beta)
{
    if (fn_setBodyState) {
        fn_setBodyState(ax, ay, az, vx, vy, vz, wind_x, wind_y, wind_z,
                       dotx, doty, dotz, omegax, omegay, omegaz,
                       yaw, pitch, roll, alpha, beta);
    }
}

void OFMInterface::simulate(double dt)
{
    if (fn_simulate) fn_simulate(dt);
}

bool OFMInterface::addLocalForceComponent(double& fx, double& fy, double& fz,
                                        double& px, double& py, double& pz)
{
    if (fn_addForce) return fn_addForce(fx, fy, fz, px, py, pz);
    return false;
}

bool OFMInterface::addLocalMomentComponent(double& mx, double& my, double& mz)
{
    if (fn_addMoment) return fn_addMoment(mx, my, mz);
    return false;
}