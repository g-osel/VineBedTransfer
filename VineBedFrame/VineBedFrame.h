/* VineBedFrame.h

Motor Control for Vine Bed Transfer
Written by: Sreela Kodali, kodali@stanford.edu

*/

#ifndef VineBedFrame_h
#define VineBedFrame_h


#define N_ACT 8   // Number of actuators, 8 motors
#define N_CMDS 3 // number of commands

#include "Arduino.h"
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

class VineBedFrame {
	public:

		//VineBedFrame(bool serial, int* cmdVal, int vines, int* vineArr, int tcws, int* tcwArr);
		VineBedFrame(bool serial);
		
		// --- OBJECTS ---  //
		Servo motorArr[N_ACT];
		cmd allCommands[N_CMDS];
		//  = {
		//   // initializeCmd("allBase", motors_BASE),
		//   // initializeCmd("allTCWTurn", motors_TCW),
		//   initializeCmd("individualMotor", motors_NONE),
		//   initializeCmd("PreLoadValues", motors_ALL),
		//   initializeCmd("executeCommand", motors_NONE)
		// };

		bool serialOn;
		const int nVines = 3;
		const int nTCWS = 3;
		const int activeVines[3] = {5, 6, 7};
		const int activeTCWS[3] = {1, 2, 3};
		const int uSCommandValues[10] = { MOTOR_NEUTRAL, 1400, 1436, 1450, 1475, MOTOR_NEUTRAL, 1600, 1686, 1736, 1750};

		// FIX: note, can make these values for nVines nTCWS the default, perhaps can pass else 
		// FIX: could even parameterize uSCommandValues and the pins
		
		// --- METHODS --- //
		void preloadValues(unsigned long commandValue);
		void executeCommand(unsigned long commandValue);
		void defaultCommand(unsigned long commandValue, int i);
		void Estop();
	
	private:

		// --- OBJECTS --- //
		const int maxParams = 4;  // Maximum parameters that can be sent in a single command
		BLEService* motorServicePtr;  // Bluetooth® Low Energy, motorized device

		// const int uS_CMDVAL_DEFAULT[10] = { MOTOR_NEUTRAL, 1400, 1436, 1450, 1475, MOTOR_NEUTRAL, 1600, 1686, 1736, 1750};
		// const int VINE_DEFAULT[3] = {5, 6, 7};				
		// const int TCW_DEFAULT[3] = {1, 2, 3};
		const int pins_CommandOUTArr[N_ACT] = { 2, 3, 4, 5, 6, 7, 8, 9 }; // these pins correspond  to different motor's input

		// which motors will be on for each command
		// tcw1, tcw2, tcw3, tcw4, base1, base2, base3, base4,
		int motors_BASE[N_ACT] = { 0, 0, 0, 0, 1, 1, 1, 1 };
		int motors_TCW[N_ACT] = { 1, 1, 1, 1, 0, 0, 0, 0 };
		int motors_ALL[N_ACT] = { 1, 1, 1, 1, 1, 1, 1, 1 };
		int motors_NONE[N_ACT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

		// --- METHODS --- //
		cmd initializeCmd(char* s, int* m);
		void initializeSystem();
		int extractByte(int length, int idx, unsigned long commandValue);

		void runMotor(unsigned long commandValue);
		void AllTCW_arr(const int* speed);
		void AllTCW(unsigned long commandValue);
		void AllVines_arr(const int* speed);
		void AllVines(unsigned long commandValue);
		void TunedVineDep();
		void LiftandReturn();
		void LiftandReturn2(unsigned long commandValue);

		void commandOverTime(unsigned long commandValue);
		void twoSpeeds(unsigned long commandValue);
		void handleHalfAndHalfSwapCommand(unsigned long commandValue);
};
#endif