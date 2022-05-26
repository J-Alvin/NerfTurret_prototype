#include "Turret.h"

// ARM DISARM TURRET
// Description:
//    Arms or Disarms the turret by turning on/off the motors. 
// Dependancies:
//    ARMING_PIN
// Input:
//    bool arm: true to arm, false to disarm
// Output: None
void Turret::armDisarmTurret(bool arm)
{
  if (arm)
  {
    int wait = this->getArmingWaitTime();
    
    // arm the turret
    this->is_armed = true;

    // Turn on arming motor
    digitalWrite(ARMING_PIN, HIGH);
          
    // wait for motor to reach speed
    if (wait > 0)
      delay(wait);
  }
  else
  {
    // Turn off arming motor
    digitalWrite(ARMING_PIN, LOW);

    this->is_armed = false;   // disarm the turret
  }  
  return;
}

// FIRE BURST
// Description:
//    Fires projectiles for length of time defiens by Turret.getBurstLength().
//    If distance > 0, turret is adjusted vertically to account for arced trajectory.
// Inputs:
//    int distance: distance of the target in cm
// Outputs: None
void Turret::fireBurst()
{
  // don't fire if not armed
  if (this->is_armed == false)
    return;
  
  unsigned long my_delay = this->getBurstLength();
  unsigned long stop_time = millis() + my_delay;
  
  while (millis() < stop_time)
  {
    // Advance Trigger Motor
    this->pTriggerStepper->runSpeed();
  }
  return;
}

// IS CENTERED
// Description:
//    Checks is the turret is centered within the given tolerance on the target
// Input: 
//    int target_h: desired horizontal position of turret
// Output:
//    bool: True if within tolerance, False otherwise
bool Turret::isCentered(float target_h)
{
  // get position errors
  float h_error = abs(this->h_pos - target_h);
  
  if (h_error > this->getHtolerance())
    return false;
  else
    return true;
}

// MOVE_H
// Description:
//    Moves the turret horizontally towards position target_h.
//    Will only move turret withing min and max positions defined
//    by MIN_H_POS and MAX_H_POS. Sets h_pos.
//    Will move turrert as far as is able.
// Input:
//    int target_h: desired h position of turret
void Turret::move_h(float target_h)
{
  // keep target pos within MIN_H_POS, MAX_H_POS
  if (target_h < MIN_H_POS)
    target_h = MIN_H_POS;
  if (target_h > MAX_H_POS)
    target_h = MAX_H_POS;
  
  // Calculate the angle delta from cur_angle
  float angle_dif = target_h - this->getHpos();
  
  // Get how many steps we need to get there
  int num_steps = STEPS_PER_DEGREE * angle_dif;
  
  // Add angles
  this->h_pos += angle_dif;
  
  // perform step
  this->pYawStepper->step(num_steps);
}
