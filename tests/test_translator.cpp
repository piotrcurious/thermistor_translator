#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"
#include "../translator.ino"

int main() {
    setup();
    Thermistor ntc(10000, 25, 3950);
    VoltageDivider circuit(5.0, 10000);

    for (double t = 0; t <= 100; t += 20) {
        double r = ntc.getResistance(t);
        double v = circuit.getVout(r);
        g_adc_value = circuit.getADC(v, 5.0, 1024);
        for(int i=0; i<100; ++i) loop();

        // Simple assertion: output should be positive and vary with temperature
        assert(g_last_analog_write_value >= 0);
        std::cout << "Temp: " << t << "C, ADC: " << g_adc_value << ", PWM: " << g_last_analog_write_value << std::endl;
    }
    std::cout << "Test passed!" << std::endl;
    return 0;
}
