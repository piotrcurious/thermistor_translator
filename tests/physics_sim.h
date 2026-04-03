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

class RCFilter {
    double R, C, Fpwm;
public:
    RCFilter(double r_ohms, double c_farads, double f_hz) : R(r_ohms), C(c_farads), Fpwm(f_hz) {}

    double getVout(double duty_cycle, double Vref) {
        return Vref * duty_cycle;
    }

    double getRipple(double duty_cycle, double Vref) {
        // Simplified ripple approximation for first-order RC filter
        // Peak-to-peak ripple: Vpp = (Vref * D * (1-D)) / (f * R * C)
        if (Fpwm <= 0 || R <= 0 || C <= 0) return 0;
        double ripple = (Vref * duty_cycle * (1.0 - duty_cycle)) / (Fpwm * R * C);
        return ripple;
    }
};

class TargetSystem {
    double R_pullup;
    Thermistor targetNtc;
public:
    TargetSystem(double r_pullup, double r0, double t0_c, double b)
        : R_pullup(r_pullup), targetNtc(r0, t0_c, b) {}

    double voltageToTemperature(double Vout, double Vref, double B, double R0, double T0_C) {
        if (Vout <= 0.001) return -999; // Error
        if (Vout >= Vref - 0.001) return 999; // Error

        double R_target = R_pullup * (Vout / Vref) / (1.0 - (Vout / Vref));

        // Inverse B-parameter equation
        double T0_K = T0_C + 273.15;
        double invT = (1.0 / T0_K) + (1.0 / B) * std::log(R_target / R0);
        return (1.0 / invT) - 273.15;
    }
};

#endif
