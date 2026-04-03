#include <iostream>
#include <vector>
#include <string>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"

// We will use namespaces to test different versions
namespace v1 { #include "../translator.ino" }
namespace v2 { #include "../translator_v2.ino" }
namespace v3 { #include "../translator_v3_switchable2.ino" }

void run_full_test(const std::string& name, void (*setup_fn)(), void (*loop_fn)(), int pwm_res, bool is_v3 = false, int select_val = HIGH) {
    std::cout << "Testing: " << name << std::endl;
    std::string log_name = "tests/data_" + name + ".csv";
    Serial.openLog(log_name);
    setup_fn();

    if (is_v3) {
        g_digital_pins[2] = select_val;
    }

    Thermistor srcNtc(10000, 25, 3950);
    VoltageDivider srcCircuit(5.0, 10000);

    RCFilter filter(10000, 10e-6, 490);
    TargetSystem target(10000, 10000, 25, 3950);

    // High resolution sweep
    for (double t = -10; t <= 110; t += 1.0) {
        double r = srcNtc.getResistance(t);
        double v_in = srcCircuit.getVout(r);
        g_adc_value = srcCircuit.getADC(v_in, 5.0, 1024);

        // Let filters settle
        for(int i=0; i<50; ++i) loop_fn();

        int pwm_out = (name == "adc_to_pwm" || name == "uno_precise") ? (int)OCR1A : g_last_analog_write_value;
        double duty = (double)pwm_out / pwm_res;
        double v_out = filter.getVout(duty, 5.0);
        double t_out = target.voltageToTemperature(v_out, 5.0, 3950, 10000, 25);

        Serial.log(millis(), t, g_adc_value, pwm_out, v_out, t_out);
    }
}

// Special wrapper for adc_to_pwm
namespace n_adc_to_pwm { #include "../adc_to_pwm.ino" }

int main() {
    run_full_test("v1", v1::setup, v1::loop, 255);
    run_full_test("v2", v2::setup, v2::loop, 255);
    run_full_test("v3_curveA", v3::setup, v3::loop, 255, true, HIGH);
    run_full_test("v3_curveB", v3::setup, v3::loop, 255, true, LOW);
    run_full_test("adc_to_pwm", n_adc_to_pwm::setup, n_adc_to_pwm::loop, 4095);

    return 0;
}
