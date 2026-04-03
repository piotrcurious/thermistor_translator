#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"
#include "../adc_to_pwm.ino"

int main() {
    Serial.openLog("tests/data_adc_to_pwm.csv");
    setup();
    Thermistor ntc(10000, 25, 3950);
    VoltageDivider circuit(5.0, 10000);

    for (double t = 0; t <= 100; t += 20) {
        double r = ntc.getResistance(t);
        double v = circuit.getVout(r);
        g_adc_value = circuit.getADC(v, 5.0, 1024);

        for(int i=0; i<100; ++i) {
            loop();
            Serial.log(millis(), g_adc_value, (int)OCR1A);
        }

        // This file uses 12-bit PWM (0-4095) in OCR1A
        assert(OCR1A >= 0 && OCR1A <= 4095);
        std::cout << "Temp: " << t << "C, ADC: " << g_adc_value << ", PWM (OCR1A): " << OCR1A << std::endl;
    }
    std::cout << "Test passed!" << std::endl;
    return 0;
}
