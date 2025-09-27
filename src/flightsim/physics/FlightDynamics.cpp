#include "FlightDynamics.h"

void FlightDynamics::computeBodyRates(const State6& s, double Fx, double Fy, double Fz,
                                    double Mx, double My, double Mz,
                                    double m, double Ixx, double Iyy, double Izz,
                                    State6& d)
{
    // s.u, s.v, s.w, s.p, s.q, s.r
    // translational eqn
    double du = s.r * s.v - s.q * s.w + Fx / m;
    double dv = s.p * s.w - s.r * s.u + Fy / m;
    double dw = s.q * s.u - s.p * s.v + Fz / m;

    // rotational eqn
    double dp = ((Iyy - Izz) / Ixx) * s.q * s.r + Mx / Ixx;
    double dq = ((Izz - Ixx) / Iyy) * s.p * s.r + My / Iyy;
    double dr = ((Ixx - Iyy) / Izz) * s.p * s.q + Mz / Izz;

    d.u = du; d.v = dv; d.w = dw; d.p = dp; d.q = dq; d.r = dr;
}

void FlightDynamics::eulerIntegrate(State6& s, double dt,
                                  double Fx, double Fy, double Fz,
                                  double Mx, double My, double Mz,
                                  double m, double Ixx, double Iyy, double Izz)
{
    State6 ds;
    computeBodyRates(s, Fx, Fy, Fz, Mx, My, Mz, m, Ixx, Iyy, Izz, ds);
    s.u += ds.u * dt;
    s.v += ds.v * dt;
    s.w += ds.w * dt;
    s.p += ds.p * dt;
    s.q += ds.q * dt;
    s.r += ds.r * dt;
}

void FlightDynamics::rk4Integrate(State6& s, double dt,
                                double Fx, double Fy, double Fz,
                                double Mx, double My, double Mz,
                                double m, double Ixx, double Iyy, double Izz)
{
    State6 k1, k2, k3, k4;
    State6 temp = s;

    // k1
    computeBodyRates(s, Fx, Fy, Fz, Mx, My, Mz, m, Ixx, Iyy, Izz, k1);

    // k2
    {
        State6 s2 = s;
        s2.u += 0.5 * k1.u * dt;
        s2.v += 0.5 * k1.v * dt;
        s2.w += 0.5 * k1.w * dt;
        s2.p += 0.5 * k1.p * dt;
        s2.q += 0.5 * k1.q * dt;
        s2.r += 0.5 * k1.r * dt;
        computeBodyRates(s2, Fx, Fy, Fz, Mx, My, Mz, m, Ixx, Iyy, Izz, k2);
    }

    // k3
    {
        State6 s3 = s;
        s3.u += 0.5 * k2.u * dt;
        s3.v += 0.5 * k2.v * dt;
        s3.w += 0.5 * k2.w * dt;
        s3.p += 0.5 * k2.p * dt;
        s3.q += 0.5 * k2.q * dt;
        s3.r += 0.5 * k2.r * dt;
        computeBodyRates(s3, Fx, Fy, Fz, Mx, My, Mz, m, Ixx, Iyy, Izz, k3);
    }

    // k4
    {
        State6 s4 = s;
        s4.u += k3.u * dt;
        s4.v += k3.v * dt;
        s4.w += k3.w * dt;
        s4.p += k3.p * dt;
        s4.q += k3.q * dt;
        s4.r += k3.r * dt;
        computeBodyRates(s4, Fx, Fy, Fz, Mx, My, Mz, m, Ixx, Iyy, Izz, k4);
    }

    // combine
    s.u += (dt / 6.0) * (k1.u + 2.0 * k2.u + 2.0 * k3.u + k4.u);
    s.v += (dt / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    s.w += (dt / 6.0) * (k1.w + 2.0 * k2.w + 2.0 * k3.w + k4.w);
    s.p += (dt / 6.0) * (k1.p + 2.0 * k2.p + 2.0 * k3.p + k4.p);
    s.q += (dt / 6.0) * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q);
    s.r += (dt / 6.0) * (k1.r + 2.0 * k2.r + 2.0 * k3.r + k4.r);
}