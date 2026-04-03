#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define A0 0
#define A1 1
#define A2 2

#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t*)(addr))
#define pgm_read_word(addr) (*(const uint16_t*)(addr))

// Mocking some AVR registers
uint8_t TCCR1A = 0;
uint8_t TCCR1B = 0;
uint16_t ICR1 = 0;
uint16_t OCR1A = 0;

#define WGM10 0
#define WGM11 1
#define WGM12 3
#define WGM13 4
#define CS10  0
#define COM1A1 7

// Mocking functions
unsigned long g_millis = 0;
uint16_t g_adc_value = 0;
uint8_t g_digital_pins[14] = {HIGH}; // Initialize all as HIGH for pull-up simulation

void pinMode(uint8_t pin, uint8_t mode) {}
void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 14) g_digital_pins[pin] = val;
}
int digitalRead(uint8_t pin) {
    if (pin < 14) return g_digital_pins[pin];
    return HIGH;
}
int analogRead(uint8_t pin) {
    return g_adc_value;
}

uint8_t g_last_analog_write_pin = 0;
int g_last_analog_write_value = 0;

void analogWrite(uint8_t pin, int val) {
    g_last_analog_write_pin = pin;
    g_last_analog_write_value = val;
}

unsigned long millis() {
    g_millis += 1; // Faster time advancement to avoid long wait loops
    return g_millis;
}

void delay(unsigned long ms) {
    g_millis += ms;
}

void delayMicroseconds(unsigned int us) {
    // For simplicity, we can ignore tiny delays or increment g_millis
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<class T, class L, class H>
T constrain(T x, L l, H h) {
  if(x < l) return l;
  if(x > h) return h;
  return x;
}

// Some of the .ino files use analogWriteResolution and analogWriteMax which are not standard for Uno
// but were used in translator_uno_precise.ino (wait, that file said for Uno but used these...)
// Actually, translator_uno_precise.ino had its own implementation of 12-bit PWM.
// But some files might use them.
void analogWriteResolution(int res) {}
int analogWriteMax() { return 4096; } // Assuming 12-bit based on the code in some files

#define static_assert(cond, msg)

#endif
