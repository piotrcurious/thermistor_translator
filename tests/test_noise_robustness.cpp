#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
#include "mock_arduino/Arduino.h"

// Test adc_to_pwm.ino's Kalman filter
namespace adc_to_pwm {
    #include "../adc_to_pwm.ino"
}

// Test translator_v3_switchable2.ino's IIR filter
namespace v3_switchable2 {
    #include "../translator_v3_switchable2.ino"
}

void test_noise_kalman(int iterations) {
    adc_to_pwm::setup();
    g_adc_value = 512;
    setADCNoise(20.0f);

    std::vector<int> results;
    // Need more iterations since loop is now non-blocking and processes every 10ms
    // With 100us delay in loop, we need 100 iterations to pass 10ms.
    for (int i = 0; i < iterations * 100; ++i) {
        adc_to_pwm::loop();
        if (i % 100 == 0)
            results.push_back((int)OCR1A);
    }

    double sum = std::accumulate(results.begin(), results.end(), 0.0);
    double mean = sum / results.size();
    double sq_sum = std::inner_product(results.begin(), results.end(), results.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / results.size() - mean * mean);

    std::cout << "Test: adc_to_pwm (Kalman)" << std::endl;
    std::cout << "  Mean: " << mean << ", Stdev: " << stdev << std::endl;
}

void test_noise_iir(int iterations) {
    v3_switchable2::setup();
    g_adc_value = 512;
    setADCNoise(20.0f);

    std::vector<int> results;
    for (int i = 0; i < iterations; ++i) {
        v3_switchable2::loop();
        results.push_back(g_last_analog_write_value);
    }

    double sum = std::accumulate(results.begin(), results.end(), 0.0);
    double mean = sum / results.size();
    double sq_sum = std::inner_product(results.begin(), results.end(), results.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / results.size() - mean * mean);

    std::cout << "Test: v3_switchable2 (IIR)" << std::endl;
    std::cout << "  Mean: " << mean << ", Stdev: " << stdev << std::endl;
}

int main() {
    srand(42);
    test_noise_kalman(1000);
    test_noise_iir(1000);
    return 0;
}
