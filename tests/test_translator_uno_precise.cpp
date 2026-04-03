#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"
#include "../translator_uno_precise.ino"

int main() {
    setup();
    Thermistor ntc(10000, 25, 3950);
    VoltageDivider circuit(5.0, 10000);

    for (double t = 0; t <= 100; t += 20) {
        double r = ntc.getResistance(t);
        double v = circuit.getVout(r);
        g_adc_value = circuit.getADC(v, 5.0, 1024);

        // Let it filter
        for(int i=0; i<100; ++i) loop();

        // For this file, it writes to OCR1A
        assert(OCR1A >= 0 && OCR1A <= 4096);
        std::cout << "Temp: " << t << "C, ADC: " << g_adc_value << ", PWM (OCR1A): " << OCR1A << std::endl;
    }
    std::cout << "Test passed!" << std::endl;
    return 0;
}
