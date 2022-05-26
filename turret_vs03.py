# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 22:06:01 2021

AUTONOMOUS TURRET
Move a turret horizontally and vertically, targetting an object. Then fire a projectile.
The turret is capable of leading the target and accounting for distance to the target

Limitations:
    Only targets objects horizontally at a fixed vertically angle
    Unable to account for movement towards and away from turret when leading

@author: Dallin Poole
@author: J. Alvin Harris
@author: Jeff Murray
@author: Thomas Koster
"""

RUN_TEST = False

import math # For calculating the needed projectile trajectory
import time # For leading the target


# DESIGN ASSUMPTIONS:
#   -Camera, Range Finder, and Barrel move synchronously and inline with the turret
#   -Turret only targets objects in the Horizontal
#   -Vertical and horizontal positions measured in degrees
#   -Negative offsets move turret to the right/down, positive to the left/up
#   -v_pos = 0 results in the turret being completely level

# CHANGE LOG:
#   - setup or initialization functions were added for each hardware interface
#   - setup functions create and initialize classes to read/write to hardware. 
#       These classes are hardware specific and may be defined by imported libraries
#   - previous function prototypes interacting with hardware were adjusted to include
#       hardware interface classes
#   - a Dummy Class is used as a placeholder for each Hardware Interface.
#       Using the dummy class will print the action and hardware accessed to the 
#       terminal
#   - AxctivateTurret: more fleched out to include stubs and logic for arming and 
#       disarming the turret
#   - A test variable was added (RUN_TEST) (line 19). When set to true the main 
#       loop will run once. This code 127-129 and 473-475 must be removed before
#       final product

# POSSIBLE UPGRADE: Target in 3D
#   -findObject returns offset_v in addition to offset_h
#   -turret is moved horizontally and vertically to point at target
#   -then range is retreived, angle calculated...
#   -no need to move turret vertically to point at target, it will be done next cycle 

# POSSIBLE UPGRADE: Pan Camera if No Target is found
#   -find object returns a failed object detection after a given amount of time
#   -the turret pans back and forth trying to find a targetable object

# Dummy Object as placeholder for Hardware Interface
# Can be eliminated once the actual hardware interfaces are included
class DummyHardwareInterface():
    name = ""
    data = ""
    def __init__(self, name="None", data="..."):
        self.name = name
        self.data = data
        print("Initializing " + self.name)
        return
    def getName(self):
        return self.name
    def read(self):
        print("Reading from " + self.name)
        return self.data
    def write(self, data="..."):
        self.data = data
        print("Writing to   " + self.name + ": " + self.data)
        return

# CONSTANTS
# Set according to hardware limitations
MIN_H_POS = -90 # Minimum Horizontal Position
MAX_H_POS =  90 # Maximum Horizontal Position
MIN_V_POS =   0 # Minimum Vertical Position
MAX_V_POS =  45 # Maximum Vertical Position
MAX_RANGE = 400 # maximum range of turret in centimeters
                # this is the maximum range of the range finder 
                # or the maximum range of the armamemt

# The Forever Loop  
def loop():
    
    # initialize variables
    h_pos = 0 # horizontal turret position
    v_pos = 0 # vertical turret position
    
    obj_velocity = 0
    offset_lead  = 0
    time0 = 0
    time1 = 1

    offset_h = 0
    h_failed = True

    offset_v = 0
    v_failed = True

    distance = -1
    
    is_launcher_active = False
    
    # initilize hardware and objects for hardware interfaces
    camera          = initCamera()          # findObject()
    rangeFinder     = initRangeFinder()     # getRange()
    verticalMotor   = initVerticalMotor()   # moveTurretVertical()
    horizontalMotor = initHorizontalMotor() # moveTurretHorizontal()
    armingHardware  = initArmingHardware()  # activateTurret()
    triggerHardware = initTriggerHardware() # fire()
    
    # calibrate
    # move turret to center horizontal 0 and vertical 0

    # ... INSERT CODE HERE ...
    
    # Find, Target, and Fire upon object
    # restart loop if unable to achieve turret position
    # restart loop if target out of range
    loopForever = True
    while (loopForever):
        # TEST RUN: loop once
        if (RUN_TEST):
            loopForever = False
        
        # Target has been shot, or no valid target available
        # Disable projectile launcher while searching for targets
        is_launcher_active = activateTurret(armingHardware, False, is_launcher_active)
        
        # Find Object, get time of object location
        offset_h, obj_velocity = findObject(camera)
        time0 = time.time()
        
        # Target Found: activate projectile launcher
        is_launcher_active = activateTurret(armingHardware, True, is_launcher_active)
        
        # Center Turret on Target: Horizontally
        h_pos, h_failed = moveTurretHorizontal(horizontalMotor, h_pos, offset_h)
        
        # If unable to complete horizontal movement, object is to far out of 
        # frame or out of maximum horizontal movement. Therefore, restart loop
        if (h_failed):
            continue
        
        # Get Distance 
        distance = getRange(rangeFinder)
        
        # If no target not within distance, restart loop
        if (distance == -1):
            continue
        
        # Compute Vertical Offset from Distance to Target
        offset_v = computeTrajectoryAngle(v_pos, distance)
        
        # Move Turret Vertically
        v_pos, v_failed = moveTurretVertical(verticalMotor, v_pos, offset_v)
        
        # If unable to move turret completely, restart loop
        if (v_failed):
            continue

        # Lead the Target
        # Move the turret horizontally to where the target will be
        time1 = time.time()
        offset_lead = (time1 - time0) * obj_velocity
        h_pos, h_failed = moveTurretHorizontal(horizontalMotor, h_pos, offset_lead)
        
        # If unable to move turret completely, restart loop
        if (h_failed):
            continue
        
        # Fire Projectile
        fire(triggerHardware, is_launcher_active)
        
        # Return turret to point at target vertically
        v_pos, v_failed = moveTurretVertical(verticalMotor, v_pos, -offset_v)
        
        # Delay ?
        
        # Continue loop
    return

# INIT CAMERA
# Description:
#   Initilizaes the camera hardware
# Inputs: None
# Outputs:
#   camera: an object used to interface with the hardware    
def initCamera():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    camera = DummyHardwareInterface("Camera")
    
    return camera

# INIT RANGE FINDER
# Description:
#   Initilizaes the range finder hardware
# Inputs: None
# Outputs:
#   rangeFinder: an object used to interface with the hardware    
def initRangeFinder():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    rangeFinder = DummyHardwareInterface("Range Finder")
    
    return rangeFinder

# INIT VERTICAL MOTOR
# Description:
#   Initilizaes the vertical motor hardware
# Inputs: None
# Outputs:
#   verticalMotor: an object used to interface with the hardware    
def initVerticalMotor():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    verticalMotor = DummyHardwareInterface("Vertical Motor")
    
    return verticalMotor

# INIT HORIZONTAL MOTOR
# Description:
#   Initilizaes the horizontal motor hardware
# Inputs: None
# Outputs:
#   horizontalMotor: an object used to interface with the hardware    
def initHorizontalMotor():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    horizontalMotor = DummyHardwareInterface("Horizontal Motor")
    
    return horizontalMotor

# INIT ARMING HARDWARE
# Description:
#   Initilizaes the arming hardware, the flywheels/accelerators that
#   launch the projectile
# Inputs: None
# Outputs:
#   armingHardware: an object used to interface with the hardware    
def initArmingHardware():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    armingHardware = DummyHardwareInterface("Arming Hardware")
    
    return armingHardware

# INIT TRIGGER HARDWARE
# Description:
#   Initilizaes the trigger hardware, the mechanism that loads a projectile into
#   the launch chamber 
# Inputs: None
# Outputs:
#   triggerHardware: an object used to interface with the hardware    
def initTriggerHardware():
    # Temporary Object used until valid code is added
    # ... INSERT CODE HERE ...
    triggerHardware = DummyHardwareInterface("Trigger Hardware")
    
    return triggerHardware

# FIND OBJECT
# Description:
#   Finds an object in the camera's view and calculates its position from the 
#   center of the frame. Also calcultes the velocity of the object
# Inputs:
#   hardwareInterface: the object/data used for controlling and interacting with 
#       the camera hardware
# Outputs: 
#   h_offset: horizontal position of object relative to the camera (degrees)
#   obj_velocity: horizontal velocity of the object in (degrees / second)
def findObject(hardwareInterface):
    offset_h     = 0
    obj_velocity = 0
    
    # find object
    # find horizontal position relative to frame center
    # find obj velocity

    # ... INSERT CODE HERE ...
    # BEGIN Temp code
    hardwareInterface.read()
    # END Temp code
    
    return offset_h, obj_velocity

# MOVE TURRET HORIZONTAL
# Description: 
#   Moves the turret horizontally by activating the horizontal position motors.
#   If unable to achieve the desired movement, the furthest obtainable offset 
#   will be used. (UNITS: all units in degrees)
# Inputs:
#   hardwareInterface: 
#       the object/data used for controlling and interacting with the 
#       horizontal motor hardware
#   h_pos    : the current horizontal position of the turret
#   offset_h : the desired change in position of the turret
#   min_h_pos: the minimum valid horizontal position of the turret
#   max_h_pos: the maximum valid horizontal position of the turret
# Outputs:
#   new_h_pos: the new horizontal position of the turret
#   failed   : a boolean indicating that the desired offset was obtained
def moveTurretHorizontal(hardwareInterface, h_pos, offset_h, min_h_pos=MIN_H_POS, max_h_pos=MAX_H_POS):
    
    # move turret horizontally

    # ... INSERT CODE HERE ...
    # BEGIN Temp code
    hardwareInterface.write()
    # END Temp code

    
    # adjust variables
    new_h_pos = h_pos + offset_h
    failed    = False
    if (new_h_pos > max_h_pos):
        new_h_pos = max_h_pos
        failed = True
    elif (new_h_pos < min_h_pos):
        new_h_pos = min_h_pos
        failed = True
        
    return new_h_pos, failed

# MOVE TURRET VERTICAL
# Description: 
#   Moves the turret vertically by activating the vertical position motors
#   If unable to achieve the desired movement, the furthest obtainable offset 
#   will be used. (UNITS: all units in degrees)
# Inputs:
#   hardwareInterface: 
#       the object/data used for controlling and interacting with the 
#       vertical motor hardware
#   v_pos    : the current vertical position of the turret
#   offset_v : the desired change in position of the turret
#   min_v_pos: the minimum valid vertical position of the turret
#   max_v_pos: the maximum valid vertical position of the turret
# Outputs:
#   new_v_pos: the new vertical position of the turret
#   failed   : a boolean indicating that the desired offset was obtained
def moveTurretVertical(hardwareInterface, v_pos, offset_v, min_v_pos=MIN_V_POS, max_v_pos=MAX_V_POS):
    
    # move turret vertically

    # ... INSERT CODE HERE ...
    # BEGIN Temp code
    hardwareInterface.write()
    # END Temp code
    
    # adjust variables
    new_v_pos = v_pos + offset_v
    failed    = False
    if (new_v_pos > max_v_pos):
        new_v_pos = max_v_pos
        failed = True
    elif (new_v_pos < min_v_pos):
        new_v_pos = min_v_pos
        failed = True
        
    return new_v_pos, failed

# GET RANGE
# Description:
#   Gets the range of the target (UNITS: centimeters).
# Inputs:
#   hardwareInterface: 
#       the object/data used for controlling and interacting with the 
#       range finder hardware
#   max_range: the maximum range the range finder can obtain or the maximum
#       range the turret can target
# Outputs:
#   distance: range to the target (0 to MAX_RANGE, -1 if no valid target in range)
def getRange(hardwareInterface, max_range=MAX_RANGE):
    distance = 0
    
    # find the distance to the target
    # return -1 if no valid distance or out of range

    # ... INSERT CODE HERE ...
    # BEGIN Temp code
    hardwareInterface.read()
    # END Temp code
    
    if (distance > max_range):
        distance = -1
        
    return distance

# COMPUTE TRAJECTORY ANGLE
# Description:
#   Using the current position and distance to target, compute the needed vertical
#   offset in order to hit the target
# Inputs:
#   v_pos: current vertical position (in degrees)
#   distance: distance to target (centimeters)
# Outputs:
#   offset_v: the needed change in vertical position (degrees)
def computeTrajectoryAngle(v_pos, distance):
    offset_v = 0
    
    # ... INSERT CODE HERE ...
    # ? mass and speed of projectile needed
    
    # compute needed angle offset
    return offset_v

# ACTIVATE TURRET
# Description:
#   Activates or deactivates the projectile launcher (ie accelerators) but does
#   not fire a projectile
# Inputs:
#   hardwareInterface: 
#       the object/data used for controlling and interacting with the 
#       arming hardware, or the accelerators
#   activate: boolean; True to activate, False to deactivate
#   current_state: the current state of the turret (bool: active/inactive)
# Outputs:
#   is_active: the current state of the launcher
def activateTurret(hardwareInterface, activate, current_state):
    is_active = current_state
    
    # do nothing if state doesn't change
    if (current_state == activate):
        return current_state

    if (activate == True):
        # activate turret
        is_active = True

        # ... INSERT CODE HERE ...
        # BEGIN Temp code
        hardwareInterface.write(data="Arming")
        # END Temp code

    else:
        # deactivate turret
        is_active = False

        # ... INSERT CODE HERE ...
        # BEGIN Temp code
        hardwareInterface.write(data="Disarming")
        # END Temp code
    
    return is_active

# FIRE
# Description:
#   Activates the firing mechanism of the turret if the launcher has been activated
# Inputs:
#   hardwareInterface: 
#       the object/data used for controlling and interacting with the 
#       trigger hardware
#   is_turret_active: a boolean indicating if the launcher has been turned on. 
#       (See activateTurret)
# Outputs: None
def fire(hardwareInterface, is_turret_active):
    
    if (is_turret_active == True):
        is_turret_active == True
        # ... INSERT CODE HERE ...
        # BEGIN Temp code
        hardwareInterface.write(data="Firing")
        # END Temp code

    return

# TEST RUN
if (RUN_TEST):
    loop()