/*
  Code to Control one vine robot base brushed DC motor 
  Godson Osele obum@stanford.edu, June 14th 2024

*/

/*---------------Libraries-----------------------------*/
#include <Arduino.h>
#include <SPI.h>

/*---------------Brushed DC Motor Defines-----------------------------*/
// #define REDLED 23
// #define BLUELED 22
// #define WAITINGLED 21
#define E1 4    //controls dir
#define PWM_pin 3  // Sets PWM (Speed)



/*---------------Brushed DC Motor Function Prototypes-----------------*/
//move robot functions
void GrowFunc(int speed);      // Grow the vine straight out
void RetractFunc(int speed);   // Retract the vine straight in
void Stop(void);               // Stop the motor
int SpeedVal(int SpeedInput);  // maps user speed from 0 to 100 to PWM value


/*---------------Overarching Command Function Prototypes-----------------*/
void executeCommand(char command, float param);

// /*---------------Brushed DC Motor State Definitions--------------------------*/
// typedef enum {
//   GROWING,
//   RETRACTING,
//   STOP
// } States_t;

/*---------------Brushed DC Motor Variables---------------------------*/


/*---------------Brushed Motor Limits--------------------------*/
typedef enum {
  MOTOR_CMD_MIN = 0,
  MOTOR_CMD_MAX = 100,
  //MAX_UNITS = 1024
} MOTOR_CMD_LIMITS;


//States_t state;

/*---------------Main Functions----------------*/
void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Wait for the serial communication to be ready
  while (!Serial) {
    ;  // Wait for the serial port to connect. Needed for native USB port only
  }

  Serial.println("Initialized.");

  pinMode(PWM_pin, OUTPUT);
  pinMode(E1, OUTPUT);
  //state = STOP;

  // Set initial motor PWM (Speed)
  analogWrite(PWM_pin, MOTOR_CMD_MIN);
}


void loop() {

  
  // if a value is received:
  if (Serial.available() > 0) {

    // Read the input string from serial
    String inputString = Serial.readStringUntil('\n');

    // Remove any whitespace characters
    inputString.trim();

    // Extract the command identifier (first character)
    char command = inputString.charAt(0);
    float param;

    // Print the command identifier
    Serial.print("Command: ");
    Serial.println(command);
    if (isDigit(command)) {
      Serial.println("Command not a letter");
    } else {
      Serial.println("Command is a letter");
      // Extract parameters
      int len = inputString.length();                      // Length of string minus 1
      int startIdx = 1;                                        // Start after the command character
      String paramStr = inputString.substring(startIdx, len);  // Get paramater in String format from a substring of input string
      if (!isDigit(paramStr[0])) {
        Serial.println("Parameter not a number");
      } else {
        param = paramStr.toFloat();
      }
    }

    executeCommand(command, param);
  }
}


/*----------------Module Functions--------------------------*/
int SpeedVal(int SpeedInput) {  // maps user speed from 0 to 100 to PWM value
  return map(SpeedInput, 0, 100, 0, 255);
}


void GrowFunc(int speed) {  //everts robot
  constrain(speed, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
  digitalWrite(E1, HIGH);
  analogWrite(PWM_pin, speed);
}

void RetractFunc(int speed) {  //retracts robot
  constrain(speed, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
  digitalWrite(E1, LOW);
  analogWrite(PWM_pin, speed);
}

void executeCommand(char command, float param) {
  // Placeholder for command execution logic
  //digitalWrite(LED_BUILTIN, HIGH);  // turn on LED value for commmand being sent
  Serial.print("Executing command ");
  Serial.println(command);


  if (command == 'E') {
    int speed = SpeedVal((int)param);
    Serial.print("Using Parameter: " + String(param) + ", Speed: " + String(speed));
    // Serial.print("Using parameter ");
    // Serial.println(String(param));
    GrowFunc(speed);
  } else if (command == 'R') {
    int speed = SpeedVal((int)param);
    // Serial.println(String(speed));
    Serial.print("Using Parameter: " + String(param) + ", Speed: " + String(speed));
    // Serial.println(String(param));
    RetractFunc(speed);
  } else if (command == 'S') {
    // Serial.print("Using parameter ");
    // Serial.println(String(param));
    Serial.print("Using Parameter: " + String(param));
    Stop();
  } else{
    Serial.print("No valid command input");
  }
}



void Stop(void) {  // stops driving motors
  analogWrite(PWM_pin, MOTOR_CMD_MIN);
}


