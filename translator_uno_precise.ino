
// Arduino code to translate analog signal read by A0 pin to PWM analog out signal on pin 9
// using a table stored in PROGMEM and interpolating in-between missing points
// with oversampling and Kalman filtering of input signal
// with 12-bit PWM resolution and fastest PWM timer mode
// for Arduino Uno or Nano

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

// Define the table of input-output values
const uint8_t table[] PROGMEM = {
  0,   // input = 0, output = 0
  25,  // input = 1, output = 25
  50,  // input = 2, output = 50
  75,  // input = 3, output = 75
  100, // input = 4, output = 100
};

// Define the number of entries in the table
const uint8_t tableSize = sizeof(table) / sizeof(table[0]);

// Define the input and output pins
const uint8_t inputPin = A0;
const uint8_t outputPin = 9;

// Define the interval for oversampling in milliseconds
const uint16_t oversamplingInterval = 10;

// Define the Kalman filter parameters
float kalmanGain = 0.01; // initial guess for the Kalman gain
float estimate = 0; // initial guess for the estimate
float errorEstimate = 1; // initial guess for the estimate error
float errorMeasure = 0.1; // measurement error

void setup() {
  
  // Set the output pin as output
  pinMode(outputPin, OUTPUT);
  
  // Configure Timer1 for 12-bit PWM resolution (0-4095)
  // Set the PWM timer to fastest mode (without prescaler)
  
    // Stop Timer1
    TCCR1A = 0;
    TCCR1B = 0;
    
    // Set Timer1 mode to Fast PWM with TOP at ICR1 (mode 14)
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    
    // Set Timer1 prescaler to no prescaling (clock frequency is F_CPU)
    TCCR1B |= (1 << CS10);
    
    // Set Timer1 TOP value to 4095 for 12-bit resolution
    ICR1 = 4095;
    
    // Set Timer1 duty cycle for pin9 to zero initially
    OCR1A = 0;
    
    // Enable Timer1 PWM output on pin9
    TCCR1A |= (1 << COM1A1);
    
}

static uint32_t adcSum = 0;
static uint16_t adcSamples = 0;
static uint32_t lastOversampleMs = 0;

void loop() {

    // ── Oversampling (Non-blocking) ──────────────────────────────────────────
    adcSum += analogRead(inputPin);
    adcSamples++;

    if (millis() - lastOversampleMs >= oversamplingInterval) {
        float measurement = (float)adcSum / adcSamples;
        adcSum = 0;
        adcSamples = 0;
        lastOversampleMs = millis();

        // ── Kalman filter ─────────────────────────────────────────────────────
        // Predict step (simplified: state is constant)
        // Update step
        kalmanGain = errorEstimate / (errorEstimate + errorMeasure);
        estimate = estimate + kalmanGain * (measurement - estimate);
        errorEstimate = (1 - kalmanGain) * errorEstimate;
    }

    // Calculate fractional table index
    float tableIndexF = estimate * (tableSize - 1) / 1023.0f;
    int tableIndex = (int)tableIndexF;

    uint16_t outputValue;
    if (tableIndex >= tableSize - 1) {
        outputValue = pgm_read_byte(&table[tableSize - 1]);
    } else {
        uint8_t y0 = pgm_read_byte(&table[tableIndex]);
        uint8_t y1 = pgm_read_byte(&table[tableIndex + 1]);
        float fraction = tableIndexF - (float)tableIndex;
        outputValue = (uint16_t)(y0 + fraction * (y1 - y0) + 0.5f);
    }

    // Scale to 12-bit if table values were 8-bit,
    // but here the table values are 0-100, and output should be 0-4095.
    // Wait, the original code didn't scale much, just used the table value.
    // Let's check the original intention. Table was 0-100. PWM was 0-4095.
    // This looks like it needs scaling or the table values should be larger.
    // Let's scale it to the full 12-bit range.
    uint16_t scaledOutput = map(outputValue, 0, 100, 0, 4095);
      
      // Write the output value to pin9 using Timer1 duty cycle register OCR1A
      OCR1A = scaledOutput;
}
