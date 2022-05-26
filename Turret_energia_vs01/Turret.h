/* Turret.h */

#ifndef Turret_h
#define Turret_h

#include <AccelStepper.h>
#include <Stepper.h>

// DEFINE MOTOR PINS: used to move turret and load projectile
const int NUM_PINS        = 4;
const int YAW_PINS[4]     = {8, 10, 9, 11};
const int TRIGGER_PINS[4] = {7, 6, 5, 4};

// DEFINE ARMING PIN: used to turn on/off gun motors
const int ARMING_PIN   = 3;
const int WAIT_TIME    = 2 * 1000; // time in millisec for arming motor to reach speed

// SETUP STEPPER MOTOR CONSTANTS
const int YAW_SPEED           = 10;
const int STEPS_PER_REV       = 2048;
const float STEPS_PER_DEGREE  = 5.68;

const int TRIGGER_SPEED       = 1000;
const int TRIGGER_MAX_SPEED   = 500;

// TURRET CONSTANTS
const float MIN_H_POS = 0;
const float MAX_H_POS = 50;
const float H_TOLERANCE = 5; // Range turret needs to be in to fire at target

const int BURST_LENGTH = 2 * 1000;

// TURRET CLASS
// Description:
//  Holds all the data, methods, and motor controls necessary for the Turret:
//    position: vertical, horizontal
//    is_firing: are we currently shooting
//    is_armed: are the guns motors on
class Turret
{
private:
  AccelStepper * pTriggerStepper;
  Stepper * pYawStepper;
  
  bool is_armed;

  int h_pos;        // horizontal position
  int h_tolerance;  // horizontal tolerance to begin firing
  int burst_length; // time in millisec of firing burst
  int wait_time;    // millis-sec, time needed for arming motors to reach speed

public:
  // Constructor
  Turret() 
    : h_pos(0), is_armed(false), h_tolerance(H_TOLERANCE),
      burst_length(BURST_LENGTH), wait_time(WAIT_TIME)
  {
    this->pTriggerStepper = new AccelStepper(NUM_PINS, TRIGGER_PINS[0], TRIGGER_PINS[1], TRIGGER_PINS[2], TRIGGER_PINS[3]);
    this->pYawStepper = new Stepper(STEPS_PER_REV, YAW_PINS[0], YAW_PINS[1], YAW_PINS[2], YAW_PINS[3]);

    // Initialize Trigger
    this->pTriggerStepper->setMaxSpeed(TRIGGER_MAX_SPEED);    
    this->pTriggerStepper->setSpeed(TRIGGER_SPEED);

    // Initialize Yaw
    // set rpm
    this->pYawStepper->setSpeed(YAW_SPEED);
    // test yaw
    this->pYawStepper->step(1024);
    this->pYawStepper->step(-1024);
    
    // Initialize Arming Pin
    pinMode(ARMING_PIN, OUTPUT);
    
    // initialize the turret as disarmed and not firing
    this->armDisarmTurret(false);
  }

  // Desctructor
  ~Turret()
  {
    delete this->pYawStepper;
    delete this->pTriggerStepper;
  }

  // getters and setters
  float getHpos() {return this->h_pos;}

  float getHtolerance() {return this->h_tolerance;}
  void setHtolerance(float tol) {this->h_tolerance = abs(tol);} 
  
  int getBurstLength() {return this->burst_length;}
  void setBurstLength(int millisec) {this->burst_length = abs(millisec);}

  int getArmingWaitTime() {return this->wait_time;}
  void setArmingWaitTime(int wait) {this->wait_time = abs(wait);}
  
  // methods
  void armDisarmTurret(bool arm=false);
  void fireBurst();
  bool isCentered(float target_h);
  void move_h(float target_h);
};

#endif
