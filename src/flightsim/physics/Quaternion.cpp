#include "Quaternion.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void QuaternionOperations::normalize(FlightQuaternion& q)
{
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 1e-14) {
        q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    }
}

void QuaternionOperations::toMatrix(const FlightQuaternion& q, double M[3][3])
{
    // body->NED rot matrisi (direction cosine)
    double w = q.w; double x = q.x; double y = q.y; double z = q.z;
    double ww = w * w, xx = x * x, yy = y * y, zz = z * z;

    M[0][0] = ww + xx - yy - zz;
    M[0][1] = 2.0 * (x * y - w * z);
    M[0][2] = 2.0 * (x * z + w * y);
    M[1][0] = 2.0 * (x * y + w * z);
    M[1][1] = ww - xx + yy - zz;
    M[1][2] = 2.0 * (y * z - w * x);
    M[2][0] = 2.0 * (x * z - w * y);
    M[2][1] = 2.0 * (y * z + w * x);
    M[2][2] = ww - xx - yy + zz;
}

void QuaternionOperations::integrate(FlightQuaternion& q, double p, double q_, double r, double dt)
{
    // Basit Euler yontemi
    double halfdt = 0.5 * dt;
    double dw = -(q.x * p + q.y * q_ + q.z * r) * halfdt;
    double dx = (q.w * p + q.y * r - q.z * q_) * halfdt;
    double dy = (q.w * q_ + q.z * p - q.x * r) * halfdt;
    double dz = (q.w * r + q.x * q_ - q.y * p) * halfdt;
    q.w += dw;
    q.x += dx;
    q.y += dy;
    q.z += dz;
    normalize(q);
}

void QuaternionOperations::toEulerZYX(const FlightQuaternion& q, double& roll, double& pitch, double& yaw)
{
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void QuaternionOperations::bodyToWorld(const double R[3][3], double bx, double by, double bz,
                                     double& wx, double& wy, double& wz)
{
    wx = R[0][0] * bx + R[0][1] * by + R[0][2] * bz;
    wy = R[1][0] * bx + R[1][1] * by + R[1][2] * bz;
    wz = R[2][0] * bx + R[2][1] * by + R[2][2] * bz;
}