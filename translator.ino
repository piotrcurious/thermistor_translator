
// Arduino code to translate analog signal read by A0 pin to PWM analog out signal on pin 9
// using a table stored in PROGMEM and interpolating in-between missing points

// Define the table of input-output values (Input is assumed linear 0-1023)
const uint8_t pwmPoints[] PROGMEM = {
  0, 25, 50, 75, 100
};

// Define the number of entries in the table
const uint8_t tableSize = sizeof(pwmPoints) / sizeof(pwmPoints[0]);

// Define the input and output pins
const uint8_t inputPin = A0;
const uint8_t outputPin = 9;

void setup() {
  // Set the output pin as output
  pinMode(outputPin, OUTPUT);
}

void loop() {
  // Read the analog value from the input pin (0-1023)
  int inputValue = analogRead(inputPin);

  // Calculate the fractional index in the table
  float tableIndexF = (float)inputValue * (tableSize - 1) / 1023.0f;
  int tableIndex = (int)tableIndexF;

  if (tableIndex >= tableSize - 1) {
    analogWrite(outputPin, pgm_read_byte(&pwmPoints[tableSize - 1]));
    return;
  }

  // Read the two surrounding values from the table
  uint8_t y0 = pgm_read_byte(&pwmPoints[tableIndex]);
  uint8_t y1 = pgm_read_byte(&pwmPoints[tableIndex + 1]);

  // Interpolate
  float fraction = tableIndexF - (float)tableIndex;
  uint8_t outputValue = (uint8_t)(y0 + fraction * (y1 - y0) + 0.5f);

  // Write the output value to the output pin as PWM (0-255)
  analogWrite(outputPin, outputValue);
}
