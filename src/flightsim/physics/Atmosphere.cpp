#include "Atmosphere.h"
#include <cmath>

void Atmosphere::get1976StandardAtmosphere(double h, double& T, double& a, double& rho, double& p)
{
    // Dikey sinirlar ekliyoruz: 0 - 20km
    // 0-11km troposfer, 11-20km basit model.

    if (h < 0) h = 0;
    if (h > 20000.0) h = 20000.0; // ornek sinir

    const double T0 = 288.15; // [K]
    const double p0 = 101325.0; // [Pa]
    const double g = 9.80665;
    const double R = 287.053;
    const double L = 0.0065; // K/m

    if (h <= 11000.0) {
        // troposfer
        double Th = T0 - L * h;
        double ratio = Th / T0;
        double expo = g / (R * L);
        double ph = p0 * std::pow(ratio, expo);
        double rhoh = ph / (R * Th);
        double ah = std::sqrt(1.4 * R * Th);
        T = Th; a = ah; p = ph; rho = rhoh;
    }
    else {
        // 11-20km stratosfer baslangici
        // model: T sabit 216.65 K, barometrik p asagida
        // T(11km)=216.65, p(11km)=22632 Pa
        double T11 = 216.65;
        double p11 = 22632.0;
        double hDelta = h - 11000.0;
        // T sabit
        T = T11;
        // basinc (p) exp form: p = p11 * exp( -g*(h-11000)/(R*T) )
        double ph = p11 * std::exp(-g * hDelta / (R * T11));
        double rhoh = ph / (R * T11);
        double ah = std::sqrt(1.4 * R * T11);
        a = ah; p = ph; rho = rhoh;
    }
}