/* VineFrame.h

Motor Control for Vine Bed Transfer
Written by: Sreela Kodali, kodali@stanford.edu

*/

#ifndef VineFrame_h
#define VineFrame_h


#define N_ACT 8   // Number of actuators, 8 motors
#define N_CMDS 3 // number of commands


#include <ArduinoBLE.h>
#include <Servo.h>

typedef enum {
  MOTOR_MIN = 1025,
  MOTOR_NEUTRAL = 1500,
  MOTOR_MAX = 2000,
  MOTOR_NEUTRAL_MID = 1513
} MOTOR_LIMITS;

struct cmd {
  char* strname;
  int* motors;
  BLEIntCharacteristic ble;
};

class VineFrame {
	public:

		VineFrame();

		Servo motorArr[N_ACT];
		cmd allCommands[N_CMDS];

		const int activeVines[nVines] = {5, 6, 7};
		const int activeTCWS[nTCWS] = {1, 2, 3};
		const bool serialOn = false;
		const int nVines = 3;
		const int nTCWS = 3;
		const int uSCommandValues[10] = { MOTOR_NEUTRAL, 1400, 1436, 1450, 1475, MOTOR_NEUTRAL, 1600, 1686, 1736, 1750};
		const int pins_CommandOUTArr[N_ACT] = { 2, 3, 4, 5, 6, 7, 8, 9 }; // these pins correspond  to different motor's input

		cmd initializeCmd(char* s, int* m);
	
	private:
		const int maxParams = 4;  // Maximum parameters that can be sent in a single command
		BLEService motorService("01D");  // Bluetooth® Low Energy, motorized device

};
#endif