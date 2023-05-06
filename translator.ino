
// Arduino code to translate analog signal read by A0 pin to PWM analog out signal on pin 9
// using a table stored in PROGMEM and interpolating in-between missing points

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

void setup() {
  // Set the output pin as output
  pinMode(outputPin, OUTPUT);
}

void loop() {
  // Read the analog value from the input pin (0-1023)
  int inputValue = analogRead(inputPin);

  // Map the input value to the table range (0-4)
  int tableIndex = map(inputValue, 0, 1023, 0, tableSize - 1);

  // Constrain the table index to avoid overflow
  tableIndex = constrain(tableIndex, 0, tableSize - 1);

  // Read the output value from the table using PROGMEM
  int outputValue = pgm_read_byte(&table[tableIndex]);

  // If the table index is not the last one, interpolate the output value with the next one
  if (tableIndex < tableSize - 1) {
    // Read the next output value from the table using PROGMEM
    int nextOutputValue = pgm_read_byte(&table[tableIndex + 1]);

    // Calculate the fraction of the input value between the two table indices (0-1)
    float fraction = (float)(inputValue - map(tableIndex, 0, tableSize - 1, 0, 1023)) / (float)(map(tableIndex + 1, 0, tableSize - 1, 0, 1023) - map(tableIndex, 0, tableSize -1 ,0 ,1023));

    // Interpolate the output value with the next one using the fraction
    outputValue = outputValue + fraction * (nextOutputValue - outputValue);
    
    // Constrain the output value to avoid overflow
    outputValue = constrain(outputValue,0 ,255);
    
    }

    // Write the output value to the output pin as PWM (0-255)
    analogWrite(outputPin,outputValue);
}
