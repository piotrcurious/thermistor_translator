#include "mock_arduino/Arduino.h"
#include "physics_sim.h"
#include "../translator_v3_switchable2.ino"

int main() {
    Serial.openLog("tests/data_v3_curveB.csv");
    setup();
    g_digital_pins[2] = LOW;
    Thermistor srcNtc(10000, 25, 3950);
    VoltageDivider srcCircuit(5.0, 10000);
    RCFilter filter(10000, 10e-6, 490);
    TargetSystem target(10000, 10000, 25, 3950);
    for (double t = -10; t <= 110; t += 1.0) {
        double r = srcNtc.getResistance(t);
        double v_in = srcCircuit.getVout(r);
        g_adc_value = srcCircuit.getADC(v_in, 5.0, 1024);
        for(int i=0; i<100; ++i) loop();
        int pwm_out = g_last_analog_write_value;
        double duty = (double)pwm_out / 255;
        double v_out = filter.getVout(duty, 5.0);
        double t_out = target.voltageToTemperature(v_out, 5.0, 3950, 10000, 25);
        Serial.log(millis(), t, g_adc_value, pwm_out, v_out, t_out);
    }
    return 0;
}
