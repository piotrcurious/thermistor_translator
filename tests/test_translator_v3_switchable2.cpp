#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"
#include "../translator_v3_switchable2.ino"

int main() {
    setup();
    Thermistor ntc(10000, 25, 3950);
    VoltageDivider circuit(5.0, 10000);

    for (int select = 0; select <= 1; ++select) {
        g_digital_pins[2] = select ? HIGH : LOW;
        std::cout << "--- SELECT: " << (select ? "HIGH" : "LOW") << " ---" << std::endl;

        for (double t = 0; t <= 100; t += 20) {
            double r = ntc.getResistance(t);
            double v = circuit.getVout(r);
            g_adc_value = circuit.getADC(v, 5.0, 1024);

            for(int i=0; i<100; ++i) loop();

            assert(g_last_analog_write_value >= 0 && g_last_analog_write_value <= 255);
            std::cout << "Temp: " << t << "C, ADC: " << g_adc_value << ", PWM: " << g_last_analog_write_value << std::endl;
        }
    }
    std::cout << "Test passed!" << std::endl;
    return 0;
}
