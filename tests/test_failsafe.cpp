#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"
#include "../translator_v3_switchable2.ino"

int main() {
    setup();

    // Normal operation
    g_adc_value = 512;
    for(int i=0; i<100; ++i) loop();
    std::cout << "Normal (512) -> PWM: " << (int)g_last_analog_write_value << std::endl;
    assert(g_last_analog_write_value < 255);

    // Disconnected (ADC = 1023)
    g_adc_value = 1023;
    for(int i=0; i<100; ++i) loop();
    std::cout << "Disconnected (1023) -> PWM: " << (int)g_last_analog_write_value << std::endl;
    assert(g_last_analog_write_value == 255);

    // Short to GND (ADC = 0)
    g_adc_value = 0;
    for(int i=0; i<100; ++i) loop();
    std::cout << "Short to GND (0) -> PWM: " << (int)g_last_analog_write_value << std::endl;
    assert(g_last_analog_write_value == 255);

    std::cout << "Failsafe test passed!" << std::endl;
    return 0;
}
