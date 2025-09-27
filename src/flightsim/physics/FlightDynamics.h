#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct State6 {
    // linear velocities in body
    double u, v, w;
    // angular rates p,q,r in body
    double p, q, r;

    State6() : u(0), v(0), w(0), p(0), q(0), r(0) {}
};

class FlightDynamics {
public:
    static void computeBodyRates(const State6& s, double Fx, double Fy, double Fz,
                               double Mx, double My, double Mz,
                               double m, double Ixx, double Iyy, double Izz,
                               State6& d);

    static void eulerIntegrate(State6& s, double dt,
                             double Fx, double Fy, double Fz,
                             double Mx, double My, double Mz,
                             double m, double Ixx, double Iyy, double Izz);

    static void rk4Integrate(State6& s, double dt,
                           double Fx, double Fy, double Fz,
                           double Mx, double My, double Mz,
                           double m, double Ixx, double Iyy, double Izz);
};