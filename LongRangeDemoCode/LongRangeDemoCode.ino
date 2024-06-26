
/*
  Code to Control one vine robot base brushed DC motor (AndyMark 2.5 in. CIM Motor 16T for CIM Sport),
  one tip clutching winch brushless DC motor (NE0 550-am) via SparkMax ESC,
   and Pressure Regulator Control for QB4TANKKZP25PSG
  Godson Osele obum@stanford.edu, June 14th 2024

  Adapted from code written by Yimeng Qin, Sreela Kodali, Brian H Do and Godson Osele
*/

/*---------------Libraries-----------------------------*/
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

/*---------------Brushed DC Motor Defines-----------------------------*/
#define IN_A 5    //controls dir
#define IN_B 4    //controls dir
#define PWM_pin 3  // Sets PWM (Speed) for Brushed motor

/*---------------Brushless DC Motor Defines-----------------------------*/
#define BL_PWM 10    // Sets PWM (Speed) for Brushless motor
Servo BL_Motor;

/*---------------Pressure Regulator Defines-----------------------------*/
#define t_SendCommands 3000    // 3 seconds
#define t_dataRate 100         // 100 ms
#define pressureCommandOUT 9  // Pin for pressure output

/*---------------Brushed DC Motor Function Prototypes-----------------*/
//move robot functions
void GrowFunc(int speed);      // Grow the vine straight out
void RetractFunc(int speed);   // Retract the vine straight in
void Stop(void);               // Stop the motor
int SpeedVal(int SpeedInput);  // maps user speed from 0 to 100 to PWM value

/*---------------Brushless DC Motor Function Prototypes-----------------*/
//move robot functions
void SpinTCW(int DutyCycle);      // Spin TCW

/*---------------Pressure Regulator Function Prototypes-----------------*/
float mapFloat(bool type, float x, int int_min, int int_max, float float_min, float float_max);  // Map analog input to presure output
void PressureCommand(float pressureCommand);

/*---------------Overarching Command Function Prototype-----------------*/
void executeCommand(char command, float param);


/*---------------Brushless DC Motor Variables---------------------------*/
typedef enum {
  MOTOR_MIN = 1025,
  MOTOR_NEUTRAL = 1500,
  MOTOR_MAX = 2000,
  MOTOR_NEUTRAL_MID = 1513
} BL_MOTOR_LIMITS;

/*---------------Brushed Motor Limits--------------------------*/
typedef enum {
  MOTOR_CMD_MIN = 0,
  MOTOR_CMD_MAX = 100,
  //MAX_UNITS = 1024
} MOTOR_CMD_LIMITS;

/*---------------Pressure Regulator Limits--------------------------*/
typedef enum {
  PRESSURE_CMD_MIN = 0,
  PRESSURE_CMD_MAX = 255,
  //MAX_UNITS = 1024
} PRESSURE_CMD_LIMITS;

/*---------------Brushless DC Motor Variables---------------------------*/
const int uSCommandValues[10] = { MOTOR_NEUTRAL, 1400, 1436, 1450, 1475, MOTOR_NEUTRAL, 1600, 1686, 1736, 1750};


/*---------------Pressure Regulator Variables---------------------------*/
const float systemMax = 15.0;  // MAX PRESSURE FOR OUR OURPUT
const int nPoints_movingAvg = 48;
int average[nPoints_movingAvg];
int count = 0;
float averagedFeedback = 0.0;
const int pressureFeedbackIN = A0;
const bool serialOn = true;
float pressureCommand = 0.0;
float pressureFeedbackProcessed = 0.0;
int pwmCommand = 0;
const float regulatorMin = 0.0;  // for mapping
const float regulatorMax = 25.0;



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
  pinMode(IN_A, OUTPUT);
  pinMode(IN_B, OUTPUT);
  pinMode(pressureCommandOUT, OUTPUT);
  BL_Motor.attach(BL_PWM);

  // Set initial motor PWM (Speed)
  analogWrite(PWM_pin, MOTOR_CMD_MIN);

  // default pressure
  analogWrite(pressureCommandOUT, PRESSURE_CMD_MIN);
  for (int i = 0; i < nPoints_movingAvg; i++) {
    average[i] = 0;
  }


  // pinMode(BLUELED, OUTPUT);
  // pinMode(REDLED, OUTPUT);



  // currentmillis = millis();
}
void loop() {


  Serial.println("Pressure Command: " + (String)(pressureCommand) + 
               ", Pressure CMD: " + (String)(pwmCommand));
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
      Serial.print("Command not a letter");
    } else {
      Serial.print("Command is a letter");
      // Extract parameters
      int len = inputString.length() - 1;                      // Length of string minus 1
      int startIdx = 1;                                        // Start after the command character
      String paramStr = inputString.substring(startIdx, len);  // Get paramater in String format from a substring of input string
      if (!isDigit(paramStr[0])) {
        Serial.print("Parameter not a number");
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
  digitalWrite(IN_A, LOW);
  digitalWrite(IN_B, HIGH);
  analogWrite(PWM_pin, speed);
}

void RetractFunc(int speed) {  //retracts robot
  constrain(speed, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
  digitalWrite(IN_A, HIGH);
  digitalWrite(IN_B, LOW);
  analogWrite(PWM_pin, speed);
}

void executeCommand(char command, float param) {
  // Placeholder for command execution logic
  //digitalWrite(LED_BUILTIN, HIGH);  // turn on LED value for commmand being sent
  Serial.print("Executing command ");
  Serial.println(command);


  if (command == 'E') {
    int speed = SpeedVal((int)param);
    Serial.print("Using parameter ");
    Serial.println(String(param));
    GrowFunc(speed);
  } else if (command == 'R') {
    int speed = SpeedVal((int)param);
    Serial.print("Using parameter ");
    Serial.println(String(param));
    RetractFunc(speed);
  } else if (command == 'S') {
    Serial.print("Using Parameter: " + String(param));
    Stop();
  } else if (command == 'P') {
    float pressureCommand = param;
    PressureCommand(pressureCommand);
  } else if (command == 'T') {
    int DC = (int)param;
    SpinTCW(DC);
  } else if (command == 'X') {
    StopAll();
  }else{
    Serial.print("No valid command input");
  }

}



void Stop(void) {  // stops driving motors
  analogWrite(PWM_pin, MOTOR_CMD_MIN);
}

void SpinTCW(int DutyCycle){ // Spin TCW
  if ((DutyCycle > 9) || (DutyCycle < 0)) {
      BL_Motor.writeMicroseconds(MOTOR_NEUTRAL);
    }
  BL_Motor.writeMicroseconds(uSCommandValues[DutyCycle]);
}

void StopAll(void){
  Stop();
  SpinTCW(5);
}

float mapFloat(bool type, float x, int int_min, int int_max, float float_min, float float_max) {
  float output;
  if (type) {  // float to int

    output = (float)(((x - float_min) * (int_max - int_min) / (float_max - float_min) + int_min));

  } else {  // int to float. in case
    output = (float)(((x - int_min) * (float_max - float_min) / (int_max - int_min) + float_min));
  }
  return output;
}

void PressureCommand(float pressureCommand) {
  if (pressureCommand > systemMax) {
    pressureCommand = systemMax;
  } else if (pressureCommand < 0) {
    pressureCommand = 0.0;
  }
  Serial.read();
  Serial.println("NEW COMMAND: " + String(pressureCommand));
  // map pressure command to DAC PWM output. float to int
  pwmCommand = mapFloat(true, pressureCommand, PRESSURE_CMD_MIN, PRESSURE_CMD_MAX, regulatorMin, regulatorMax);
  if (pwmCommand > PRESSURE_CMD_MAX) {
    pwmCommand = PRESSURE_CMD_MAX;
  } else if (pwmCommand < PRESSURE_CMD_MIN) {
    pwmCommand = PRESSURE_CMD_MIN;
  }
  analogWrite(pressureCommandOUT, pwmCommand);
  delay(t_SendCommands);
  delay(t_dataRate);
}
