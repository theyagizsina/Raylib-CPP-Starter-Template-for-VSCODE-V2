#pragma once

struct FlightQuaternion {
    double w, x, y, z;

    FlightQuaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    FlightQuaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};

class QuaternionOperations {
public:
    static void normalize(FlightQuaternion& q);
    static void toMatrix(const FlightQuaternion& q, double M[3][3]);
    static void integrate(FlightQuaternion& q, double p, double q_, double r, double dt);
    static void toEulerZYX(const FlightQuaternion& q, double& roll, double& pitch, double& yaw);
    static void bodyToWorld(const double R[3][3], double bx, double by, double bz,
                          double& wx, double& wy, double& wz);
};