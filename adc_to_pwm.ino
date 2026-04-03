// Arduino code to translate analog signal read by A0 pin to PWM on pin 9
// using a PROGMEM table with linear interpolation between entries,
// oversampling and scalar Kalman filtering of the ADC input,
// 12-bit PWM via direct Timer1 register manipulation.
// Target: Arduino Uno / Nano (ATmega328P)

#include <avr/pgmspace.h>

// ── Lookup table ─────────────────────────────────────────────────────────────
// Maps ADC input range (0–1023) to 12-bit PWM output (0–4095).
// Entries are assumed evenly spaced across the ADC range.
// MUST be uint16_t (12-bit values) read with pgm_read_word().
static const uint16_t TABLE[] PROGMEM = {
       0,  // input ≈    0  →  0 %
    1024,  // input ≈  256  → 25 %
    2048,  // input ≈  512  → 50 %
    3072,  // input ≈  768  → 75 %
    4095,  // input ≈ 1023  → 100 %
};

static const uint8_t  TABLE_SIZE  = sizeof(TABLE) / sizeof(TABLE[0]);
static const uint16_t PWM_MAX     = 4095;   // ICR1 TOP for 12-bit Fast PWM
static const uint16_t ADC_MAX     = 1023;

// ── Pins ─────────────────────────────────────────────────────────────────────
static const uint8_t INPUT_PIN  = A0;
static const uint8_t OUTPUT_PIN = 9;   // OC1A — Timer1 channel A

// ── Oversampling window (ms) ─────────────────────────────────────────────────
static const uint16_t OVERSAMPLE_MS = 10;

// ── Scalar Kalman filter ──────────────────────────────────────────────────────
// 1-D model: state is the "true" ADC value.
// Predict:  x_k|k-1 = x_{k-1}           P_k|k-1 = P_{k-1} + Q
// Update:   K        = P_k|k-1 / (P_k|k-1 + R)
//           x_k      = x_k|k-1 + K*(z - x_k|k-1)
//           P_k      = (1-K) * P_k|k-1        ← use P_k|k-1, not P_{k-1}
//
// Q — process noise variance: larger → tracks signal changes faster
// R — measurement noise variance: larger → smoother but slower response
static float kf_x = 0.0f;    // state estimate
static float kf_P = 100.0f;  // estimate variance (large initial value → fast pull-in)
static const float KF_Q = 1.0f;   // tune for your signal dynamics
static const float KF_R = 8.0f;   // tune for your ADC noise level

// ── Loop period (ms) ─────────────────────────────────────────────────────────
static const uint16_t LOOP_PERIOD_MS = 20;

// ─────────────────────────────────────────────────────────────────────────────
// Table lookup with linear interpolation.
// adcVal is the filtered ADC estimate (0–1023 range, float).
// Returns a 12-bit PWM value (0–4095).
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t lookupInterpolate(float adcVal) {
    // Hard clamp to valid range
    if (adcVal <= 0.0f)   return pgm_read_word(&TABLE[0]);
    if (adcVal >= ADC_MAX) return pgm_read_word(&TABLE[TABLE_SIZE - 1]);

    // Convert ADC value to a fractional table index
    float  fi  = adcVal * (TABLE_SIZE - 1) / (float)ADC_MAX;
    uint8_t i  = (uint8_t)fi;                // lower table index
    if (i >= TABLE_SIZE - 1) i = TABLE_SIZE - 2;  // safety clamp

    uint16_t lo   = pgm_read_word(&TABLE[i]);
    uint16_t hi   = pgm_read_word(&TABLE[i + 1]);
    float    frac = fi - (float)i;           // 0.0 … 1.0

    return (uint16_t)(lo + frac * (int16_t)(hi - lo) + 0.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(OUTPUT_PIN, OUTPUT);

    // Configure Timer1: Fast PWM, mode 14 (TOP = ICR1), no prescaler
    // NOTE: analogWriteResolution() is NOT available on AVR targets.
    //       12-bit PWM is achieved purely through direct register setup.
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1A |= (1 << WGM11);                   // WGM11
    TCCR1B |= (1 << WGM13) | (1 << WGM12);    // WGM13, WGM12  → mode 14
    TCCR1B |= (1 << CS10);                     // prescaler = 1 (no prescaling)
                                               // f_PWM = 16 MHz / 4096 ≈ 3.9 kHz

    ICR1  = PWM_MAX;    // TOP = 4095
    OCR1A = 0;          // initial duty = 0 %

    TCCR1A |= (1 << COM1A1);  // non-inverting PWM on OC1A (pin 9)
}

// ─────────────────────────────────────────────────────────────────────────────
static uint32_t adcSum   = 0;
static uint16_t adcCount = 0;
static uint32_t lastOversampleMs = 0;

void loop() {
    uint32_t loopStart = millis();

    // ── Oversampling (Non-blocking) ──────────────────────────────────────────
    // Accumulate ADC readings continuously. Process every OVERSAMPLE_MS.
    adcSum += analogRead(INPUT_PIN);
    adcCount++;

    if (millis() - lastOversampleMs >= OVERSAMPLE_MS) {
        float measurement = (adcCount > 0)
                            ? (float)adcSum / adcCount
                            : kf_x;

        adcSum = 0;
        adcCount = 0;
        lastOversampleMs = millis();

        // ── Kalman filter ─────────────────────────────────────────────────────
        // Predict step
        float P_pred = kf_P + KF_Q;

        // Update step
        float K  = P_pred / (P_pred + KF_R);
        kf_x     = kf_x + K * (measurement - kf_x);
        kf_P     = (1.0f - K) * P_pred;
    }

    // ── Table lookup + interpolation ─────────────────────────────────────────
    // We do this every loop to ensure PWM is always set, although kf_x
    // only changes every OVERSAMPLE_MS.
    uint16_t pwmVal = lookupInterpolate(kf_x);

    // ── Write to Timer1 directly ─────────────────────────────────────────────
    OCR1A = pwmVal;

    // Maintain a small loop delay to avoid running too fast in the simulation
    // and to simulate real-world ADC sample rate limits.
    delayMicroseconds(100);
}
