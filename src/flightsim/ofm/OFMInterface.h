#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

class OFMInterface {
private:
#ifdef _WIN32
    HMODULE hModule;
#else
    void* hModule;
#endif

    typedef void  (*ofm_initialize_t)();
    typedef void  (*ofm_set_command_t)(int, float);
    typedef void  (*ofm_set_atmosphere_t)(double, double, double, double, double, double, double, double);
    typedef void  (*ofm_set_current_mass_state_t)(double, double, double, double, double, double, double);
    typedef void  (*ofm_set_current_state_body_axis_t)(
        double ax, double ay, double az,
        double vx, double vy, double vz,
        double wind_x, double wind_y, double wind_z,
        double dotx, double doty, double dotz,
        double omegax, double omegay, double omegaz,
        double yaw, double pitch, double roll,
        double alpha, double beta
        );
    typedef void  (*ofm_simulate_t)(double);
    typedef bool  (*ofm_add_local_force_component_t)(double&, double&, double&, double&, double&, double&);
    typedef bool  (*ofm_add_local_moment_component_t)(double&, double&, double&);

    ofm_initialize_t               fn_init;
    ofm_set_command_t              fn_setCmd;
    ofm_set_atmosphere_t           fn_setAtmos;
    ofm_set_current_mass_state_t   fn_setMass;
    ofm_set_current_state_body_axis_t fn_setBodyState;
    ofm_simulate_t                 fn_simulate;
    ofm_add_local_force_component_t fn_addForce;
    ofm_add_local_moment_component_t fn_addMoment;

public:
    OFMInterface();
    ~OFMInterface();

    bool initialize();
    void shutdown();

    void init();
    void setCommand(int cmd, float value);
    void setAtmosphere(double altitude, double T, double a, double rho, double p,
                      double windX, double windY, double windZ);
    void setMassState(double mass, double cmX, double cmY, double cmZ,
                     double moiX, double moiY, double moiZ);
    void setBodyState(double ax, double ay, double az,
                     double vx, double vy, double vz,
                     double wind_x, double wind_y, double wind_z,
                     double dotx, double doty, double dotz,
                     double omegax, double omegay, double omegaz,
                     double yaw, double pitch, double roll,
                     double alpha, double beta);
    void simulate(double dt);
    bool addLocalForceComponent(double& fx, double& fy, double& fz,
                               double& px, double& py, double& pz);
    bool addLocalMomentComponent(double& mx, double& my, double& mz);
};