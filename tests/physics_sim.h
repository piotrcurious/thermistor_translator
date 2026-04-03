#ifndef PHYSICS_SIM_H
#define PHYSICS_SIM_H

#include <cmath>

class Thermistor {
    double R0, T0, B;
public:
    Thermistor(double r0, double t0_c, double b) : R0(r0), T0(t0_c + 273.15), B(b) {}

    double getResistance(double temp_c) {
        double T = temp_c + 273.15;
        return R0 * std::exp(B * (1.0/T - 1.0/T0));
    }
};

class VoltageDivider {
    double Vin, R_pullup;
public:
    VoltageDivider(double vin, double r_pullup) : Vin(vin), R_pullup(r_pullup) {}

    double getVout(double R_thermistor) {
        return Vin * R_thermistor / (R_pullup + R_thermistor);
    }

    int getADC(double Vout, double Vref, int resolution) {
        int val = (int)(Vout / Vref * resolution + 0.5);
        if (val < 0) return 0;
        if (val >= resolution) return resolution - 1;
        return val;
    }
};

#endif
