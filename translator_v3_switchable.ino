#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

constexpr uint8_t INPUT_PIN   = A0;
constexpr uint8_t OUTPUT_PIN  = 9;
constexpr uint8_t SELECT_PIN  = 2;   // INPUT_PULLUP: HIGH = curve A, LOW = curve B

constexpr uint16_t ADC_MAX = 1023;
constexpr uint8_t  PWM_MAX = 255;

// Failsafe configuration: detect sensor disconnection/short
constexpr uint16_t FAILSAFE_ADC_MIN = 5;
constexpr uint16_t FAILSAFE_ADC_MAX = 1018;
constexpr uint8_t  FAILSAFE_PWM     = 255; // Default to full speed on failure

struct TransferCurve {
  const uint16_t* adcPoints;
  const uint8_t*  pwmPoints;
  size_t          size;
};

// Curve A
const uint16_t adcPointsA[] PROGMEM = {
  0, 128, 256, 512, 768, 900, 1023
};

const uint8_t pwmPointsA[] PROGMEM = {
  0, 10, 35, 120, 200, 235, 255
};

// Curve B
const uint16_t adcPointsB[] PROGMEM = {
  0, 200, 400, 600, 800, 1023
};

const uint8_t pwmPointsB[] PROGMEM = {
  0, 0, 40, 140, 220, 255
};

constexpr TransferCurve CURVE_A { adcPointsA, pwmPointsA, sizeof(adcPointsA) / sizeof(adcPointsA[0]) };
constexpr TransferCurve CURVE_B { adcPointsB, pwmPointsB, sizeof(adcPointsB) / sizeof(adcPointsB[0]) };

static_assert((sizeof(adcPointsA) / sizeof(adcPointsA[0])) == (sizeof(pwmPointsA) / sizeof(pwmPointsA[0])),
              "Curve A arrays must have the same length");
static_assert((sizeof(adcPointsB) / sizeof(adcPointsB[0])) == (sizeof(pwmPointsB) / sizeof(pwmPointsB[0])),
              "Curve B arrays must have the same length");
static_assert((sizeof(adcPointsA) / sizeof(adcPointsA[0])) >= 2, "Curve A needs at least two points");
static_assert((sizeof(adcPointsB) / sizeof(adcPointsB[0])) >= 2, "Curve B needs at least two points");

#if defined(ARDUINO_ARCH_AVR)
static inline uint16_t readU16_P(const uint16_t* p) { return pgm_read_word(p); }
static inline uint8_t  readU8_P (const uint8_t*  p) { return pgm_read_byte(p); }
#else
static inline uint16_t readU16_P(const uint16_t* p) { return *p; }
static inline uint8_t  readU8_P (const uint8_t*  p) { return *p; }
#endif

// Optional smoothing for noisy ADC readings.
// Set FILTER_SHIFT to 0 to disable.
constexpr uint8_t FILTER_SHIFT = 2;
static bool adcFilterInitialized = false;
static uint16_t filteredAdc = 0;

static uint16_t smoothAdc(uint16_t sample)
{
  if (FILTER_SHIFT == 0) {
    return sample;
  }

  if (!adcFilterInitialized) {
    filteredAdc = sample;
    adcFilterInitialized = true;
    return sample;
  }

  const int32_t delta = (int32_t)sample - (int32_t)filteredAdc;
  filteredAdc = (uint16_t)((int32_t)filteredAdc + (delta >> FILTER_SHIFT));
  return filteredAdc;
}

static uint8_t interpolateTransfer(const TransferCurve& curve, uint16_t adc)
{
  const uint16_t firstX = readU16_P(&curve.adcPoints[0]);
  const uint16_t lastX  = readU16_P(&curve.adcPoints[curve.size - 1]);

  if (adc <= firstX) {
    return readU8_P(&curve.pwmPoints[0]);
  }
  if (adc >= lastX) {
    return readU8_P(&curve.pwmPoints[curve.size - 1]);
  }

  for (size_t i = 0; i < curve.size - 1; ++i) {
    const uint16_t x0 = readU16_P(&curve.adcPoints[i]);
    const uint16_t x1 = readU16_P(&curve.adcPoints[i + 1]);

    if (adc >= x0 && adc <= x1) {
      const uint8_t y0 = readU8_P(&curve.pwmPoints[i]);
      const uint8_t y1 = readU8_P(&curve.pwmPoints[i + 1]);

      if (x1 == x0) {
        return y0; // protection against malformed data
      }

      const uint32_t span = (uint32_t)(x1 - x0);
      const uint32_t pos  = (uint32_t)(adc - x0);
      const int32_t dy    = (int32_t)y1 - (int32_t)y0;

      int32_t y = (int32_t)y0 + (dy * (int32_t)pos + (int32_t)(span / 2)) / (int32_t)span;

      if (y < 0)   y = 0;
      if (y > PWM_MAX) y = PWM_MAX;

      return (uint8_t)y;
    }
  }

  return readU8_P(&curve.pwmPoints[curve.size - 1]);
}

// Simple debounce for the curve selector.
constexpr unsigned long SELECT_DEBOUNCE_MS = 20;
static uint8_t lastSelectReading = HIGH;
static uint8_t stableSelectState = HIGH;
static unsigned long lastSelectChangeMs = 0;

static const TransferCurve& readSelectedCurve()
{
  const uint8_t reading = digitalRead(SELECT_PIN);

  if (reading != lastSelectReading) {
    lastSelectReading = reading;
    lastSelectChangeMs = millis();
  }

  if ((millis() - lastSelectChangeMs) >= SELECT_DEBOUNCE_MS) {
    stableSelectState = reading;
  }

  return (stableSelectState == HIGH) ? CURVE_A : CURVE_B;
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

  uint8_t pwm;
  if (adc < FAILSAFE_ADC_MIN || adc > FAILSAFE_ADC_MAX) {
    pwm = FAILSAFE_PWM;
  } else {
    const TransferCurve& curve = readSelectedCurve();
    pwm = interpolateTransfer(curve, adc);
  }

  analogWrite(OUTPUT_PIN, pwm);
}
