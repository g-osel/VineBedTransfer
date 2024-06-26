/* 
VineBedFrame library
Motor control for Vine bed transfer
Written by: Sreela Kodali, kodali@stanford.edu

*/

#include <VineBedFrame.h>
#include <ArduinoBLE.h>

VineBedFrame motorSystem(false);

void setup() {
  // do nothing
}

void loop() {
    // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  if (central) {   // if a central is connected to peripheral:
    while (central.connected()) {     // while the central is still connected to peripheral:
      for (int i = 0; i < N_CMDS; ++i) { // poll through the commands
        cmd c = motorSystem.allCommands[i];
        if ((c.ble).written()) {

          char* n = c.strname;
          unsigned long x = (c.ble).value();

          // INPUT: 8 HEX values via BLE         HOW MANY MOTORS RUN: 8 MOTORS
          // What it does: Specify speed/dir command for all 8 motors in 8-digit HEX value
          if (n == "PreLoadValues") {
            if (motorSystem.serialOn) {
              Serial.println("preload");
            }
            motorSystem.preloadValues(x);

          } else if (n == "executeCommand") {
            if (motorSystem.serialOn) {
              Serial.println("executeCommand");
            }
            motorSystem.executeCommand(x);
          } 

          else {
            if (motorSystem.serialOn) {
              Serial.println("default");
            }
            motorSystem.defaultCommand(x, i);
          }
          delay(500);
        }
      }
    }
  }
}
