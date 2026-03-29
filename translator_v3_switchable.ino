#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

constexpr uint8_t INPUT_PIN   = A0;
constexpr uint8_t OUTPUT_PIN  = 9;
constexpr uint8_t SELECT_PIN  = 2;   // LOW = curve B, HIGH = curve A

// Curve A
const uint16_t adcPointsA[] PROGMEM = {
  0,   128, 256, 512, 768, 900, 1023
};

const uint8_t pwmPointsA[] PROGMEM = {
  0,   10,  35,  120, 200, 235, 255
};

// Curve B
const uint16_t adcPointsB[] PROGMEM = {
  0,   200, 400, 600, 800, 1023
};

const uint8_t pwmPointsB[] PROGMEM = {
  0,   0,   40,  140, 220, 255
};

constexpr uint8_t SIZE_A = sizeof(adcPointsA) / sizeof(adcPointsA[0]);
constexpr uint8_t SIZE_B = sizeof(adcPointsB) / sizeof(adcPointsB[0]);

static_assert(SIZE_A == (sizeof(pwmPointsA) / sizeof(pwmPointsA[0])),
              "Curve A arrays must have the same length");
static_assert(SIZE_B == (sizeof(pwmPointsB) / sizeof(pwmPointsB[0])),
              "Curve B arrays must have the same length");
static_assert(SIZE_A >= 2 && SIZE_B >= 2, "Each curve needs at least two points");

#if defined(ARDUINO_ARCH_AVR)
static inline uint16_t readU16_P(const uint16_t* p) { return pgm_read_word(p); }
static inline uint8_t  readU8_P (const uint8_t*  p) { return pgm_read_byte(p); }
#else
static inline uint16_t readU16_P(const uint16_t* p) { return *p; }
static inline uint8_t  readU8_P (const uint8_t*  p) { return *p; }
#endif

constexpr uint8_t FILTER_SHIFT = 2; // 1/4 smoothing; set to 0 to disable
static bool filterInitialized = false;
static uint16_t filteredAdc = 0;

static uint16_t smoothAdc(uint16_t sample)
{
  if (FILTER_SHIFT == 0) {
    return sample;
  }

  if (!filterInitialized) {
    filteredAdc = sample;
    filterInitialized = true;
    return sample;
  }

  int32_t delta = (int32_t)sample - (int32_t)filteredAdc;
  filteredAdc = (uint16_t)((int32_t)filteredAdc + (delta >> FILTER_SHIFT));
  return filteredAdc;
}

static uint8_t interpolateTransfer(const uint16_t* adcPoints,
                                   const uint8_t* pwmPoints,
                                   uint8_t tableSize,
                                   uint16_t adc)
{
  const uint16_t firstX = readU16_P(&adcPoints[0]);
  const uint16_t lastX  = readU16_P(&adcPoints[tableSize - 1]);

  if (adc <= firstX) {
    return readU8_P(&pwmPoints[0]);
  }
  if (adc >= lastX) {
    return readU8_P(&pwmPoints[tableSize - 1]);
  }

  for (uint8_t i = 0; i < tableSize - 1; ++i) {
    const uint16_t x0 = readU16_P(&adcPoints[i]);
    const uint16_t x1 = readU16_P(&adcPoints[i + 1]);

    if (adc >= x0 && adc <= x1) {
      const uint8_t y0 = readU8_P(&pwmPoints[i]);
      const uint8_t y1 = readU8_P(&pwmPoints[i + 1]);

      if (x1 == x0) {
        return y0; // malformed table protection
      }

      const uint32_t span = (uint32_t)(x1 - x0);
      const uint32_t pos  = (uint32_t)(adc - x0);
      const int32_t dy    = (int32_t)y1 - (int32_t)y0;

      int32_t y = (int32_t)y0 + (dy * (int32_t)pos + (int32_t)(span / 2)) / (int32_t)span;

      if (y < 0)   y = 0;
      if (y > 255) y = 255;

      return (uint8_t)y;
    }
  }

  return readU8_P(&pwmPoints[tableSize - 1]);
}

void setup()
{
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(SELECT_PIN, INPUT_PULLUP);
}

void loop()
{
  const uint16_t rawAdc = analogRead(INPUT_PIN);
  const uint16_t adc = smoothAdc(rawAdc);

  const bool useCurveA = (digitalRead(SELECT_PIN) == HIGH);

  const uint8_t pwm = useCurveA
    ? interpolateTransfer(adcPointsA, pwmPointsA, SIZE_A, adc)
    : interpolateTransfer(adcPointsB, pwmPointsB, SIZE_B, adc);

  analogWrite(OUTPUT_PIN, pwm);
}
