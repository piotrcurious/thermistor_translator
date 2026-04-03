#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include "mock_arduino/Arduino.h"
#include "physics_sim.h"

// Define a test structure
struct TestConfig {
    std::string name;
    std::string ino_path;
    int pwm_res;
    bool is_v3;
    int select_val;
};

void generate_standalone_test(const TestConfig& config) {
    std::string cpp_path = "tests/run_" + config.name + ".cpp";
    std::ofstream ofs(cpp_path);
    ofs << "#include \"mock_arduino/Arduino.h\"\n";
    ofs << "#include \"physics_sim.h\"\n";
    ofs << "#include \"../" << config.ino_path << "\"\n\n";
    ofs << "int main() {\n";
    ofs << "    Serial.openLog(\"tests/data_" << config.name << ".csv\");\n";
    ofs << "    setup();\n";
    if (config.is_v3) {
        ofs << "    g_digital_pins[2] = " << (config.select_val == HIGH ? "HIGH" : "LOW") << ";\n";
    }
    ofs << "    Thermistor srcNtc(10000, 25, 3950);\n";
    ofs << "    VoltageDivider srcCircuit(5.0, 10000);\n";
    ofs << "    RCFilter filter(10000, 10e-6, 490);\n";
    ofs << "    TargetSystem target(10000, 10000, 25, 3950);\n";
    ofs << "    for (double t = -10; t <= 110; t += 1.0) {\n";
    ofs << "        double r = srcNtc.getResistance(t);\n";
    ofs << "        double v_in = srcCircuit.getVout(r);\n";
    ofs << "        g_adc_value = srcCircuit.getADC(v_in, 5.0, 1024);\n";
    ofs << "        for(int i=0; i<100; ++i) loop();\n";
    ofs << "        int pwm_out = " << ((config.ino_path == "adc_to_pwm.ino" || config.ino_path == "translator_uno_precise.ino") ? "(int)OCR1A" : "g_last_analog_write_value") << ";\n";
    ofs << "        double duty = (double)pwm_out / " << config.pwm_res << ";\n";
    ofs << "        double v_out = filter.getVout(duty, 5.0);\n";
    ofs << "        double t_out = target.voltageToTemperature(v_out, 5.0, 3950, 10000, 25);\n";
    ofs << "        Serial.log(millis(), t, g_adc_value, pwm_out, v_out, t_out);\n";
    ofs << "    }\n";
    ofs << "    return 0;\n";
    ofs << "}\n";
    ofs.close();
}

int main() {
    std::vector<TestConfig> tests = {
        {"v1", "translator.ino", 255, false, HIGH},
        {"v2", "translator_v2.ino", 255, false, HIGH},
        {"v3_switchable_A", "translator_v3_switchable.ino", 255, true, HIGH},
        {"v3_switchable_B", "translator_v3_switchable.ino", 255, true, LOW},
        {"v3_switchable2_A", "translator_v3_switchable2.ino", 255, true, HIGH},
        {"v3_switchable2_B", "translator_v3_switchable2.ino", 255, true, LOW},
        {"adc_to_pwm", "adc_to_pwm.ino", 4095, false, HIGH},
        {"uno_precise", "translator_uno_precise.ino", 4095, false, HIGH}
    };

    for (const auto& config : tests) {
        generate_standalone_test(config);
        std::string binary = "tests/bin_" + config.name;
        std::string compile_cmd = "g++ -I tests/mock_arduino -I tests/mock_arduino/avr tests/run_" + config.name + ".cpp -o " + binary;
        if (system(compile_cmd.c_str()) == 0) {
            system(binary.c_str());
            // std::remove(binary.c_str());
            // std::remove(("tests/run_" + config.name + ".cpp").c_str());
        }
    }
    return 0;
}
