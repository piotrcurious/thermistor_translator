#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

constexpr uint8_t INPUT_PIN  = A0;
constexpr uint8_t OUTPUT_PIN = 9;
constexpr uint16_t ADC_MAX   = 1023;

// Failsafe configuration: detect sensor disconnection/short
constexpr uint16_t FAILSAFE_ADC_MIN = 5;
constexpr uint16_t FAILSAFE_ADC_MAX = 1018;
constexpr uint8_t  FAILSAFE_PWM     = 255; // Default to full speed on failure

// Explicit transfer curve:
// ADC breakpoints on the X axis, PWM values on the Y axis.
// Keep both arrays the same length and sorted by ADC value.
const uint16_t adcPoints[] PROGMEM = {
  0,   128, 256, 512, 768, 900, 1023
};

const uint8_t pwmPoints[] PROGMEM = {
  0,   10,  35,  120, 200, 235, 255
};

constexpr uint8_t TABLE_SIZE = sizeof(adcPoints) / sizeof(adcPoints[0]);
static_assert(TABLE_SIZE == (sizeof(pwmPoints) / sizeof(pwmPoints[0])),
              "adcPoints and pwmPoints must have the same length");
static_assert(TABLE_SIZE >= 2, "Need at least two table points");

#if defined(ARDUINO_ARCH_AVR)
static inline uint16_t readU16_P(const uint16_t* p) { return pgm_read_word(p); }
static inline uint8_t  readU8_P (const uint8_t*  p) { return pgm_read_byte(p); }
#else
static inline uint16_t readU16_P(const uint16_t* p) { return *p; }
static inline uint8_t  readU8_P (const uint8_t*  p) { return *p; }
#endif

// Simple IIR low-pass filter to reduce ADC jitter.
// Set FILTER_SHIFT to 0 to disable smoothing.
constexpr uint8_t FILTER_SHIFT = 2; // 1/4 smoothing
static bool filterInitialized = false;
static uint16_t filteredAdc = 0;

static uint16_t smoothAdc(uint16_t sample)
{
  if (!filterInitialized) {
    filteredAdc = sample;
    filterInitialized = true;
    return sample;
  }

  if (FILTER_SHIFT == 0) {
    return sample;
  }

  int32_t delta = (int32_t)sample - (int32_t)filteredAdc;
  filteredAdc = (uint16_t)((int32_t)filteredAdc + (delta >> FILTER_SHIFT));
  return filteredAdc;
}

static uint8_t interpolateTransfer(uint16_t adc)
{
  // Clamp to the table ends.
  const uint16_t x0_first = readU16_P(&adcPoints[0]);
  const uint16_t xN_last  = readU16_P(&adcPoints[TABLE_SIZE - 1]);

  if (adc <= x0_first) {
    return readU8_P(&pwmPoints[0]);
  }
  if (adc >= xN_last) {
    return readU8_P(&pwmPoints[TABLE_SIZE - 1]);
  }

  // Find the segment [i, i+1] containing adc.
  uint8_t i = 0;
  for (; i < TABLE_SIZE - 1; ++i) {
    uint16_t x0 = readU16_P(&adcPoints[i]);
    uint16_t x1 = readU16_P(&adcPoints[i + 1]);
    if (adc >= x0 && adc <= x1) {
      uint8_t y0 = readU8_P(&pwmPoints[i]);
      uint8_t y1 = readU8_P(&pwmPoints[i + 1]);

      if (x1 == x0) {
        return y0; // avoid divide-by-zero if table is malformed
      }

      // Integer linear interpolation with rounding:
      // y = y0 + (y1 - y0) * (adc - x0) / (x1 - x0)
      const uint32_t span = (uint32_t)(x1 - x0);
      const uint32_t pos  = (uint32_t)(adc - x0);
      const int32_t  dy   = (int32_t)y1 - (int32_t)y0;

      int32_t y = (int32_t)y0 + (dy * (int32_t)pos + (int32_t)(span / 2)) / (int32_t)span;

      if (y < 0)   y = 0;
      if (y > 255) y = 255;

      return (uint8_t)y;
    }
  }

  // Fallback, should not be reached if table is sorted.
  return readU8_P(&pwmPoints[TABLE_SIZE - 1]);
}

void setup()
{
  pinMode(OUTPUT_PIN, OUTPUT);
}

void loop()
{
  const uint16_t rawAdc = analogRead(INPUT_PIN);
  const uint16_t adc    = smoothAdc(rawAdc);

  uint8_t pwm;
  if (adc < FAILSAFE_ADC_MIN || adc > FAILSAFE_ADC_MAX) {
    pwm = FAILSAFE_PWM;
  } else {
    pwm = interpolateTransfer(adc);
  }

  analogWrite(OUTPUT_PIN, pwm);
}
