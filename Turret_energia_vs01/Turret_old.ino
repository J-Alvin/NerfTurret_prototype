// Arduino side code for Automatic Nerf Turret
// Data is received via a UART determining which direction the target
// is in. The arduino moves the turret and fires accordingly

// INCLUDES
#include "Turret.h"

// LOOP GLOBALS
Turret myTurret = Turret();
float target_h   = 0;      // h position of target

// SETUP
// Description:
//    Code to run once at the start of execution
void setup() 
{
  // initialize globals
  target_h = 0;

  // Start the UART
  // Opens serial connection on baud 115200
  
  Serial.begin(115200);
  Serial.println("Setup");

}

// LOOP
void loop() 
{
  // disarm until target is found
  myTurret.armDisarmTurret(false);  
  
  // get UART input, wait inifintely
  // will receive data in the form: float target_h
  while(Serial.available() == 0) 
  {
    // no op
  }

  // Read the float. This is a locking function that we could improve.
  target_h = Serial.parseFloat();
  
  // Debug for sanity.
  Serial.println(target_h);
      
  // if target found, arm and move turret
  myTurret.armDisarmTurret(true);
    
  myTurret.move_h(target_h);
    
  // if centered on target fire
  if (myTurret.isCentered(target_h))      
    myTurret.fireBurst();      
}
