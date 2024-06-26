/* VineBedFrame.cpp

Motor Control for Vine Bed Transfer
Written by: Sreela Kodali, kodali@stanford.edu

*/

#include "Arduino.h"
#include "pins_arduino.h"
#include "VineBedFrame.h"

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
---------------      Constructor   ---------------
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/ 
VineBedFrame::VineBedFrame(bool serial) {
	
	serialOn = serial; // false to turn serial off
  BLEService motorService("01D");
  motorServicePtr = &(motorService);
	initializeSystem();
}


/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
---------------      PUBLIC METHODS   ---------------
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


/* -- FUNCTION: ESTOP ----------------
---- electronically turns off all motors */
void VineBedFrame::Estop() {
  for (int i = 0; i < N_ACT; ++i) {
    motorArr[i].writeMicroseconds(MOTOR_NEUTRAL);
  }
}
// ------  END OF FUNCTION ESTOP ------ //


/* -- FUNCTION: PRELOAD VALUES ----------------
---- pass in 8 hexadecimal digits via BLE.
---- A (or any Letter) causes immediate stop */

void VineBedFrame::preloadValues(unsigned long commandValue) {
  
  // create and initialize
  int commands[N_ACT];
  bool sendCommand = true;
  for (int k = 0; k < N_ACT; ++k) {
    commands[k] = 0;
  }

  // extract values
  for (int k = 0; k < N_ACT; ++k) {
    commands[k] = extractByte(8, k, commandValue);
    // if any of the digits sent are A or greater, stop
    if (commands[k] >= 10 or commands[k] < 1) {
      sendCommand = false;
      break;
    }
  }
  // if no A / stop command sent, pass along command values
  if (sendCommand) {
    for (int k = 0; k < N_ACT; ++k) {
      int idx = 5; // neutral

      // for motors 6 and 8, flip the values
      if (commands[k] != 0) {
        if ( (k == 5) || (k == 7)) {
          idx = 5 + (5 - commands[k]);
        } else {
          idx = commands[k];
        }
      }
      // Serial.println(commands[k]);
      motorArr[k].writeMicroseconds(uSCommandValues[idx]);
    }
  } else { // otherwise stop 
    Estop();
  }
}
// ------  END OF FUNCTION preloadValues ------ //


/* ------  FUNCTION executeCommand ------
A (formerly 'K' in serial) - individual motor. Takes 2 params: motor index + speed/dir value
B (formerly 'B' in serial) - allVines or allBases. Takes 1 param: speed/dir
C (formerly 'T' in serial) - allTCW. Takes 1 param: speed/dir
D - tunedVineDep. No params. Runs vines with pre-specified values.
E - estop. No params. Stops all motors
F (formerly 'L' in serial) - lift and return. No params. */

void VineBedFrame::executeCommand(unsigned long commandValue) {
  
  int command[maxParams];

  for (int i = 0; i < maxParams; ++i) {
    command[i] = extractByte(maxParams, i, commandValue);

    switch(command[i]) {

      // E-STOP
      case 14: //E,  IF any 'E' is DETECTED, E-STOP.  E IS 14 in HEX
        if (serialOn) { 
          Serial.println("Estop");
        }
        Estop();
        break;

      // TUNEDVINEDEP
      case 13: // IF any 'D' is DETECTED, TUNEDVINEDEP. D IS 13 in HEX
        if (serialOn) { 
          Serial.println("TunedVineDep");
        }
        TunedVineDep();
        break;

      case 15: // IF any 'F' is DETECTED, LIFTANDRETURN. F IS 15
        if (serialOn) { 
          Serial.println("LiftAndReturn");
        }
        LiftandReturn2(commandValue);
        //LiftandReturn();
        break;

      case 12: // IF any 'C' is DETECTED, AllTCW. C is 12
        if (serialOn) { 
          Serial.println("allTcw");
        }
        AllTCW(commandValue);
        break;
      
      case 11: // IF any 'B' is DETECTED, AllVines (like allBases). B is 11
        if (serialOn) { 
          Serial.println("allVines");
        }
        AllVines(commandValue);
        break;

      case 10: // IF any 'A' is DETECTED, runMotor. A is 10
        if (serialOn) { 
          Serial.println("runMotor");
        }
        runMotor(commandValue);
        break;

//      default:
//        Estop();
//       break;
    }

    //
  }
}

// ------  END OF FUNCTION executeCommand ------ //


/* -- FUNCTION: defaultCommand ----------------
---- DEFAULT BEHAVIOR FOR allBase, allTCWTurn,
---- allBaseTCWTurn, individualMotors.
---- Send speed idx to the group of motors activated 
---- HighByte is motor idx, low byte is speed idx */

void VineBedFrame::defaultCommand(unsigned long commandValue, int i) {
    // DEFAULT BEHAVIOR FOR allBase, allTCWTurn, allBaseTCWTurn, individualMotors
	unsigned long x = commandValue;
    unsigned long z = (x & 0b11110000) >> 4;
    x = (x & 0b00001111);
    
    int y = 5;
    // Serial.println(x, HEX);

    // if value > 16, change motorArr to make sure motor of MSB turns on
    if (z) {
        for (int j = 0; j < N_ACT; ++j) {
          if (z - 1 == j) {
             (allCommands[i]).motors[j] = 1;
          } else {
             (allCommands[i]).motors[j] = 0;
          }
        }
     }
     
     // otherwise read LSB as motor command
     if (x >= 0 && x < 10) {
         y = (int)x;
         //Serial.println(y);
     }

     // for motors that are on as per motorArr, pass commands. otherwise off
     for (int j = 0; j < N_ACT; ++j) {
       if ((allCommands[i]).motors[j]) {
          if (y != 0) {
             if ( (j == 5) || (j == 7)) { // flip for motors 6 and 8
                 y = 5 + (5 - y);
              } 
           }
           motorArr[j].writeMicroseconds(uSCommandValues[y]);
           
        } else {
            motorArr[j].writeMicroseconds(MOTOR_NEUTRAL);
          }
     }
}

// ------  END OF FUNCTION defaultCommand ------ //


/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
---------------      PRIVATE METHODS   ---------------
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/ 


/* -- FUNCTION: initializeCmd ----------------
---- initialize Cmd structure by passing in char* name,
---- and a binary int array m that indicates which motors
---- are on for this command. Also creates a corresponding
---- BLE characteristic with the char* name  */


cmd VineBedFrame::initializeCmd(char* s, int* m) {
  BLEIntCharacteristic bleCharacteristic("2A59", BLEWrite);  // 2A59, analog output
  BLEDescriptor des("2901", s);                              // adding user description (2901) for the characteristic
  bleCharacteristic.addDescriptor(des);
  bleCharacteristic.writeValue(0);
  cmd c{ s, m, bleCharacteristic };
  return c;
}

// ------  END OF FUNCTION initializeCmd ------ //

/* -- FUNCTION: initializeSystem ----------------
---- void setup() actions are completed whenever 
---- VineBedFrame object is created */

void VineBedFrame::initializeSystem() {
	if (serialOn) {
    Serial.begin(9600);
    while (!Serial);
  }
  
  if (!BLE.begin()) { // begin initialization
    if (serialOn) { 
      Serial.println("starting Bluetooth® Low Energy module failed!");
    }
    while (1);
  }
  // set advertised local name and service:
  BLE.setLocalName("NanoBLE_Vine");
  BLE.setAdvertisedService(*motorServicePtr);

  // connect each motor to PWM output and set to neutral
  for (int i = 0; i < N_ACT; ++i) {
    motorArr[i].attach(pins_CommandOUTArr[i]);  
    motorArr[i].writeMicroseconds(MOTOR_NEUTRAL);
  }

  //add characteristics to service
  for (int i = 0; i < N_CMDS; ++i) {
    (*motorServicePtr).addCharacteristic(((allCommands[i]).ble));  // add the characteristic to the service
  }
  BLE.addService(*motorServicePtr);  // add service
  BLE.advertise();               // start advertising

  if (serialOn) {
    Serial.println("Initialized.");
  }
}

// ------  END OF FUNCTION initializeSystem ------ //


// ------  FUNCTION: Extracting Bytes: extract right digits from hexadecimal BLE values ------ //
// pass in either 3 or 8
// length is how many bytes you want to send
int VineBedFrame::extractByte(int length, int idx, unsigned long commandValue) {

  if (idx > (length - 1)) {
    return -1; // -1 is error
  } else {
    int valueArr[length];
    if (length == 2) {
      valueArr[0] = (commandValue & 0xF0) >> 4;
      valueArr[1] = (commandValue & 0x0F) >> 0;
    } else if (length == 4) {
      valueArr[0] = (commandValue & 0x00F0) >> 4;
      valueArr[1] = (commandValue & 0x000F) >> 0;
      valueArr[2] = (commandValue & 0xF000) >> 12;
      valueArr[3] = (commandValue & 0x0F00) >> 8;
    } else if (length == 3) {
      valueArr[0] = (commandValue & 0x000F) >> 0;
      valueArr[1] = (commandValue & 0xF000) >> 12;
      valueArr[2] = (commandValue & 0x0F00) >> 8;
    } else if (length == 8) {
      valueArr[0] = (commandValue & 0x000000F0) >> 4;
      valueArr[1] = (commandValue & 0x0000000F) >> 0;
      valueArr[2] = (commandValue & 0x0000F000) >> 12;
      valueArr[3] = (commandValue & 0x00000F00) >> 8;
      valueArr[4] = (commandValue & 0x00F00000) >> 20;
      valueArr[5] = (commandValue & 0x000F0000) >> 16;
      valueArr[6] = (commandValue & 0xF0000000) >> 28;
      valueArr[7] = (commandValue & 0x0F000000) >> 24;    
    }
    return valueArr[idx];
  }
}
// ------  END OF FUNCTION: Extracting Bytes  ------ //


/*-- FUNCTION: runMotor ----------------
---- Turn each motor individually
---- in BLE, send as: [ A, MOTOR IDX, SPEED/DIR ]
---- MOTOR IDX is 1-8. flip for motor6 and 8 accounted for */

void VineBedFrame::runMotor(unsigned long commandValue) {
  // This function sets the specified motor to the given speed
  int n = 3;   // 2 param, 1 cmd = 3
  int command[n];
  for (int i = 0; i < n; ++i) {
    command[i] = extractByte(n, i, commandValue);
  }
  int speedIdx = command[2];
  int motorIdx = command[1]-1;
  
  // for motors 6 and 8, flip the values
  if (speedIdx != 0) {
    if ( (motorIdx == 5) || (motorIdx == 7)) {
      speedIdx = 5 + (5 - speedIdx);
    } 
  }

  if (serialOn) {
    Serial.println(motorIdx + 1); 
    Serial.println(speedIdx);
  }
  int speed = uSCommandValues[speedIdx];
  motorArr[motorIdx].writeMicroseconds(speed);
}

// ------  END OF FUNCTION: runMotor  ------ //


/*-- FUNCTION: AllTCW_arr ----------------
---- Control all TCWs simultaneously
---- TCWs are motors 1, 2, 3, 4 */

void VineBedFrame::AllTCW_arr(const int* speed) {
  // send array commands only if array right length
//  int l = (sizeof(speed) / sizeof(speed[0]));
//  if (l == N_ACT / 2) {
    if (1) {
    //int speed = uSCommandValues[speedIdx];
    for (int i = 0; i < nTCWS; ++i) {
      if (serialOn) {
        Serial.println(String(activeTCWS[i]) + ", " + String(speed[i]));  
      }
      motorArr[activeTCWS[i]-1].writeMicroseconds(speed[i]);
    }
  }
}
// ------  END OF FUNCTION: AllTCW_arr  ------ //



/*-- FUNCTION: AllTCW ----------------
---- Control all TCWs simultaneously, same command
---- TAKES FULL HEXADECIMAL INPUT FROM BLE */

void VineBedFrame::AllTCW(unsigned long commandValue) {
  int n = 2;   // 1 param, 1 cmd = 2
  int command[n];
  for (int i = 0; i < n; ++i) {
    command[i] = extractByte(n, i, commandValue);
  }
  int speedIdx = command[1];
  if (serialOn) {
    Serial.println("TCWs");
    Serial.println(speedIdx);
  }
  int speed = uSCommandValues[speedIdx];
  for (int i = 0; i < nTCWS; ++i) {
    motorArr[activeTCWS[i]-1].writeMicroseconds(speed);
  }
}
// ------  END OF FUNCTION: AllTCW  ------ //


/*-- FUNCTION: AllVines_arr ----------------
---- Control all Vine Bases simultaneously
---- gets const int* array with explicit microsecond values, not speed idx
---- Flip accounted for */

void VineBedFrame::AllVines_arr(const int* speed) {
    // send array commands only if array right length
   
//  int l = (sizeof(*speed) / sizeof(speed[0]));
//  if (serialOn) {
//    Serial.println(l);
//   }
//  if (l == N_ACT / 2) {
    if (1) {
    for (int i = 0; i < nVines; ++i) {
      int s = speed[i];
      if ((activeVines[i] == 6) || (activeVines[i] == 8)) { // if motors 6 and 8, flip the value
        s = MOTOR_NEUTRAL_MID + (MOTOR_NEUTRAL_MID - s);
      }
      if (serialOn) {
        Serial.println(String(activeVines[i]) + ", " + String(s));  
      }
      motorArr[activeVines[i]-1].writeMicroseconds(s);
    }
  }
}
// ------  END OF FUNCTION: AllVines_arr  ------ //


/*-- FUNCTION: AllVines ----------------
---- Control all Vine Bases simultaneously, same command
---- TAKES FULL HEXADECIMAL INPUT FROM BLE. Flip accounted for */

void VineBedFrame::AllVines(unsigned long commandValue) {
  int n = 2;   // 1 param, 1 cmd = 2
  int command[n];
  for (int i = 0; i < n; ++i) {
    command[i] = extractByte(n, i, commandValue);
  }
  
  if (serialOn) {
      Serial.println("Vines");
      Serial.println(command[1]);
    } 
  for (int i = 0; i < nVines; ++i) {
    int speedIdx = command[1];
    int s = uSCommandValues[speedIdx];
      // for motors 6 and 8, flip the values
    if (speedIdx != 0) {
      if ( (activeVines[i] == 6) || (activeVines[i] == 8)) {
        //speedIdx = 5 + (5 - speedIdx);
        s = MOTOR_NEUTRAL_MID + (MOTOR_NEUTRAL_MID - s);
      } 
    }
//    if (serialOn) {
//      AASerial.println(activeVines[i]); 
//      Serial.println(speedIdx);
//    }
    int speed = uSCommandValues[speedIdx];
    motorArr[activeVines[i]-1].writeMicroseconds(speed);
  }
}
// ------  END OF FUNCTION: AllVines  ------ //


/*-- FUNCTION: LiftandReturn ----------------
---- Initiate Lift and Return Sequence
---- Lifting subject from bed and putting back down with preset speeds */

void VineBedFrame::LiftandReturn() {
  int arraysz = N_ACT / 2;
  // speed are chosen based on the expected ratio relationship of the rotational velocities
  // of the base and TCW per their respective radii. The "delta" from motor neutral is based
  // this ratio.
  // I.E. TCW speed = NEUTRAL + 150 -> Base speed = NEUTRAL - (150*(R_TCW/R_base))

  const int TCWSpeedLift[arraysz] = {1436, 1436, 1436, 1436 };    // TCW speeds during lift. value < 1500 = cw
  const int TCWSpeedReturn[arraysz] = { 1600, 1600, 1600, 1600 };  // TCW speeds during return. value > 1500 = ccw
  
  const int BaseSpeedLift[arraysz] = { 1245, 1245, 1245, 1245 };    // Base speeds during lift. Vines are retracting, hence, value < 1500
  const int BaseSpeedReturn[arraysz] = { 1791, 1791, 1791, 1791 };    // Base speeds during return. Vines are extending, value > 1500
  
  int duration = 5;                                                 // duration of lift and return respectively
  int carryTime = 3;

  // LIFT: TCWs are turning clockwise (values < 1500) and vines are retracting on the base (values < 1500)
  AllTCW_arr(TCWSpeedLift);
  AllVines_arr(BaseSpeedLift);
  delay(duration * 1000);  // Convert duration of lift to seconds

  // Set all motors back to neutral
  Estop();
  delay(carryTime * 1000);  // 3 second delay between commands

  // Return: TCWs are turning counter clockwise (values > 1500) and vines are extending from the base (values > 1500)
  AllTCW_arr(TCWSpeedReturn);
  AllVines_arr(BaseSpeedReturn);
  delay(duration * 1000);  // Convert duration of return to seconds

  // Set all motors back to neutral
  Estop();
}

// ------  END OF FUNCTION: LiftandReturn  ------ //


/*-- FUNCTION: LiftandReturn2 ----------------
---- Instead of a preset sequence like in LiftAndReturn, allows
---- for sending command1 to all tcws, and command2 to all vine bases
---- Allows for execution of LiftAndReturn sequence but step by step */

void VineBedFrame::LiftandReturn2(unsigned long commandValue) {
    // 2 param, 1 cmd = 3
    // MSB (command), mid byte (tcw speed), LSB (base speed)

    if (serialOn) {
      Serial.println(commandValue, HEX);
    }
    // shifting 3 byte value one byte to the right so only
    // 2 bytes and LSB is tcwSpeed
    unsigned long tcwIdx = ((commandValue & 0xFF00) >> 12); // mid byte 
    unsigned long vineIdx = ((commandValue & 0xFF00) >> 8);// lsb byte

    AllTCW(tcwIdx);
    AllVines(vineIdx);
}

// ------  END OF FUNCTION: LiftandReturn2  ------ //

  
/*-- FUNCTION: TunedVineDep ----------------
---- Function deploys all vines simultaneously with unique hard coded speeds.
---- Written to address initial growth of vine down and under subject */

void VineBedFrame::TunedVineDep() {
  int arraysz = N_ACT / 2;
  const int BaseSpeedDep[arraysz] = { 1600, 1600, 1600, MOTOR_NEUTRAL };  // Base speeds during initial deployment
  AllVines_arr(BaseSpeedDep);
}
// ------  END OF FUNCTION: TunedVineDep  ------ //


/*-- FUNCTION: commandOverTime ---------------- 
---- pass in 3 values. Motoridx = high byte,
---- speed idx = mid byte, duration = low byte */

void VineBedFrame::commandOverTime(unsigned long commandValue) {
  int motorIndex = extractByte(3, 0, commandValue);  // Extract motor index from high byte
  int speedIdx = extractByte(3, 1, commandValue);    // Extract speed index from mid byte
  int duration = extractByte(3, 2, commandValue);            // Extract duration from low byte

  if (motorIndex < N_ACT && speedIdx >= 0 && speedIdx < 10) {
    int speed = uSCommandValues[speedIdx];
    motorArr[motorIndex].writeMicroseconds(speed);
    delay(duration * 1000);  // Convert duration to milliseconds
    motorArr[motorIndex].writeMicroseconds(MOTOR_NEUTRAL);
  }
}
// ------  END OF FUNCTION commandOverTime ------ //

/*-- FUNCTION: twoSpeeds ---------------- 
---- pass in 3 values. Speed1 = high byte,
---- speed2 = mid byte, duration = low byte 
---- all the motors are run in this command */

void VineBedFrame::twoSpeeds(unsigned long commandValue) {
  int speedIdx1 = extractByte(3, 0, commandValue);  // Extract first speed index from high byte
  int speedIdx2 = extractByte(3, 1, commandValue);   // Extract second speed index from mid byte
  int duration = extractByte(3, 2, commandValue);           // Extract duration from low byte

  if (speedIdx1 >= 0 && speedIdx1 < 10 && speedIdx2 >= 0 && speedIdx2 < 10) {
    int speed1 = uSCommandValues[speedIdx1];
    int speed2 = uSCommandValues[speedIdx2];

    // write speed1 for duration
    for (int j = 0; j < N_ACT; ++j) {
      motorArr[j].writeMicroseconds(speed1);
    }
    delay(duration * 1000);  // Convert duration to milliseconds

    // stop motors for duration
    Estop();
    delay(duration * 1000);  // Neutral duration

    // write speed2 for duration
    for (int j = 0; j < N_ACT; ++j) {
      motorArr[j].writeMicroseconds(speed2);
    }
    delay(duration * 1000);  // Second speed duration

    // stop motors
    Estop();
  }
}
// ------ END OF FUNCTION twoSpeeds ------ //

/*-- FUNCTION: handleHalfAndHalfSwapCommand ---------------- 
---- pass in 3 values. Speed1 = high byte,
---- speed2 = mid byte, duration = low byte 
---- all the motors are run in this command.
---- after duration, speeds are swapped for the groups */

void VineBedFrame::handleHalfAndHalfSwapCommand(unsigned long commandValue) {
  //expected 3 values delivered as hexadecimal

  int speed1Idx = extractByte(3, 0, commandValue);//From most significant bit
  int speed2Idx = extractByte(3, 1, commandValue); // From 'mid'bit
  int duration = extractByte(3, 2, commandValue);// OR 

  int speed1 = uSCommandValues[speed1Idx];
  int speed2 = uSCommandValues[speed2Idx];

  // Set first half of motors to speed1 and second half to speed2
  for (int i = 0; i < N_ACT / 2; ++i) {
    motorArr[i].writeMicroseconds(speed1);
  }
  for (int i = N_ACT / 2; i < N_ACT; ++i) {
    motorArr[i].writeMicroseconds(speed2);
  }
  delay(duration * 1000);  // Convert duration to seconds

  // Set all motors back to neutral
  Estop();
  delay(3000);  // 3 second delay between commands

  // Swap the speeds: Set first half to speed2 and second half to speed1
  for (int i = 0; i < N_ACT / 2; ++i) {
    motorArr[i].writeMicroseconds(speed2);
  }
  for (int i = N_ACT / 2; i < N_ACT; ++i) {
    motorArr[i].writeMicroseconds(speed1);
  }
  delay(duration * 1000);  // Convert duration to seconds

  // Set all motors back to neutral
  Estop();
}

// ------  END OF FUNCTION handleHalfAndHalfSwapCommand  ------ //




