/*
  Pressure Regulator Control for QB4TANKKZP25PSG
  Sreela Kodali kodali@stanford.edu, June 8th 2024

  Adapted from code written by Yimeng Qin
*/

#define t_SendCommands 3000 // 3 seconds
#define t_dataRate 100 // 100 ms

// Define system max pressure constants
const float systemMax1 = 6; // MAX PRESSURE FOR REGULATOR 1
const float systemMax2 = 6; // MAX PRESSURE FOR REGULATOR 2

const int nPoints_movingAvg = 48;
int average[nPoints_movingAvg];
int count = 0;
float averagedFeedback1 = 0.0;
float averagedFeedback2 = 0.0;

const int pressureCommandOUT1 = 11; // Output pin for Regulator 1
const int pressureFeedbackIN1 = A0;  // Input pin for Regulator 1
const int pressureCommandOUT2 = 10; // Output pin for Regulator 2
const int pressureFeedbackIN2 = A1;  // Input pin for Regulator 2
const bool serialOn = true;

float pressureCommand1 = 0.0;
float pressureCommand2 = 0.0;
float pressureFeedbackProcessed1 = 0.0;
float pressureFeedbackProcessed2 = 0.0;
int pwmCommand1 = 0;
int pwmCommand2 = 0;

const float regulatorMin1 = 0.0;  // Min range for Regulator 1
const float regulatorMax1 = 10.0; // Max range for Regulator 1 (Shoulders)
const float regulatorMin2 = 0.0;  // Min range for Regulator 2
const float regulatorMax2 = 6.0; // Max range for Regulator 2 (Knees)

typedef enum {
  PRESSURE_CMD_MIN = 0,
  PRESSURE_CMD_MAX = 255,
  MAX_UNITS = 1024
} PRESSURE_CMD_LIMITS;

void setup() {
  if (serialOn) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Initialized.");
  }
  pinMode(pressureCommandOUT1, OUTPUT);
  pinMode(pressureCommandOUT2, OUTPUT);

  // Default commands
  analogWrite(pressureCommandOUT1, PRESSURE_CMD_MIN);
  analogWrite(pressureCommandOUT2, PRESSURE_CMD_MIN);

  for (int i = 0; i < nPoints_movingAvg; i++ ) {
    average[i] = 0;
  }
}

void loop() {
  // Check and receive pressure commands
  if (Serial.available() > 0) {
    // Read the entire input from the serial port
    String inputString = Serial.readStringUntil('\n'); // Read until newline

    // Parse the input string for two float numbers
    int commaIndex = inputString.indexOf(','); // Look for the comma
    if (commaIndex > 0) { // Ensure there's a comma in the string
      String command1String = inputString.substring(0, commaIndex);
      String command2String = inputString.substring(commaIndex + 1);

      pressureCommand1 = command1String.toFloat();
      pressureCommand2 = command2String.toFloat();

      // Clamp values for Regulator 1
      if (pressureCommand1 > systemMax1) {
        pressureCommand1 = systemMax1;
      } else if (pressureCommand1 < 0) {
        pressureCommand1 = 0.0;
      }

      // Clamp values for Regulator 2
      if (pressureCommand2 > systemMax2) {
        pressureCommand2 = systemMax2;
      } else if (pressureCommand2 < 0) {
        pressureCommand2 = 0.0;
      }

      Serial.println("NEW COMMANDS: Regulator 1: " + String(pressureCommand1) + ", Regulator 2: " + String(pressureCommand2));

      // Map pressure commands to DAC PWM outputs.
      pwmCommand1 = mapFloat(true, pressureCommand1, PRESSURE_CMD_MIN, PRESSURE_CMD_MAX, regulatorMin1, regulatorMax1);
      pwmCommand2 = mapFloat(true, pressureCommand2, PRESSURE_CMD_MIN, PRESSURE_CMD_MAX, regulatorMin2, regulatorMax2);

      // Constrain PWM commands
      pwmCommand1 = constrain(pwmCommand1, PRESSURE_CMD_MIN, PRESSURE_CMD_MAX);
      pwmCommand2 = constrain(pwmCommand2, PRESSURE_CMD_MIN, PRESSURE_CMD_MAX);

      // Write PWM output to the regulators
      analogWrite(pressureCommandOUT1, pwmCommand1);
      analogWrite(pressureCommandOUT2, pwmCommand2);
      
      delay(t_SendCommands);
    }
  }

  // Measure raw value of pressure feedback and map it for both regulators
  int rawFeedback1 = analogRead(pressureFeedbackIN1);
  int rawFeedback2 = analogRead(pressureFeedbackIN2);
  
  // Update moving average
  average[count] = (rawFeedback1 + rawFeedback2) / 2; // For simplicity, averaging both
  count = (count + 1) % nPoints_movingAvg; // Wrap-around condition
  float sum = 0.0;

  for (int i = 0; i < nPoints_movingAvg; i++ ) {
    sum += average[i];
  }
  averagedFeedback1 = (float)(rawFeedback1) / MAX_UNITS * regulatorMax1;
  averagedFeedback2 = (float)(rawFeedback2) / MAX_UNITS * regulatorMax2;

  Serial.println("Pressure Command 1: " + String(pressureCommand1) + 
                 ", PWM Command 1: " + String(pwmCommand1) + 
                 ", Pressure Measured 1: " + String(averagedFeedback1) + ", Pressure Command 2: " + String(pressureCommand2) + 
                 ", PWM Command 2: " + String(pwmCommand2) + 
                 ", Pressure Measured 2: " + String(averagedFeedback2));
  // Serial.println("Pressure Command 2: " + String(pressureCommand2) + 
  //                ", PWM Command 2: " + String(pwmCommand2) + 
  //                ", Pressure Measured 2: " + String(averagedFeedback2));

  delay(t_dataRate);
}

// The mapping function remains unchanged
float mapFloat(bool type, float x, int int_min, int int_max, float float_min, float float_max) {
  float output;
  if (type) { 
    output = (float) (((x - float_min) * (int_max - int_min) / (float_max - float_min) + int_min));
  } else {
    output = (float) (((x - int_min) * (float_max - float_min) / (int_max - int_min) + float_min));
  }
  return output;
}