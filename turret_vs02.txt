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

import numpy as np # For calculating the needed projectile trajectory
import time        # For leading the target

# DESIGN ASSUMPTIONS:
#   -Camera, Range Finder, and Barrel move synchronously and inline with the turret
#   -Turret only targets objects in the Horizontal
#   -Vertical and horizontal positions measured in degrees
#   -Negative offsets move turret to the right/down, positive to the left/up
#   -v_pos = 0 results in the turret being completely level

# POSSIBLE UPGRADE: Target in 3D
#   -findObject returns offset_v in addition to offset_h
#   -turret is moved horizontally and vertically to point at target
#   -then range is retreived, angle calculated...
#   -no need to move turret vertically to point at target, it will be done next cycle 

# POSSIBLE UPGRADE: Pan Camera if No Target is found
#   -find object returns a failed object detection after a given amount of time
#   -the turret pans back and forth trying to find a targetable object

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
    
    # initialize
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
    
    # calibrate
    # move turret to center horizontal 0 and vertical 0

    # ... INSERT CODE HERE ...
    
    # Find, Target, and Fire upon object
    # restart loop if unable to achieve turret position
    # restart loop if target out of range
    while (True):
        
        # Target has been shot, or no valid target available
        # Disable projectile launcher while searching for targets
        is_launcher_active = activateTurret(False)
        
        # Find Object, get time of object location
        offset_h, obj_velocity = findObject()
        time0 = time.time()
        
        # Target Found: activate projectile launcher
        is_launcher_active = activateTurret(True)
        
        # Center Turret on Target: Horizontally
        h_pos, h_failed = moveTurretHorizontal(h_pos, offset_h)
        
        # If unable to complete horizontal movement, object is to far out of 
        # frame or out of maximum horizontal movement. Therefore, restart loop
        if (h_failed):
            continue
        
        # Get Distance 
        distance = getRange()
        
        # If no target not within distance, restart loop
        if (distance == -1):
            continue
        
        # Compute Vertical Offset from Distance to Target
        offset_v = computeTrajectoryAngle(v_pos, distance)
        
        # Move Turret Vertically
        v_pos, v_failed = moveTurretVertical(v_pos, offset_v)
        
        # If unable to move turret completely, restart loop
        if (v_failed):
            continue

        # Lead the Target
        # Move the turret horizontally to where the target will be
        time1 = time.time()
        offset_lead = (time1 - time0) * obj_velocity
        h_pos, h_failed = moveTurretHorizontal(h_pos, offset_lead)
        
        # If unable to move turret completely, restart loop
        if (h_failed):
            continue
        
        # Fire Projectile
        fire(is_launcher_active)
        
        # Return turret to point at target vertically
        v_pos, v_failed = moveTurretVertical(v_pos, -offset_v)
        
        # Delay ?
        
        # Continue loop
    return

# FIND OBJECT
# Description:
#   Finds an object in the camera's view and calculates its position from the 
#   center of the frame. Also calcultes the velocity of the object
# Inputs: None
# Outputs: 
#   h_offset: horizontal position of object relative to the camera
#   obj_velocity: horizontal velocity of the object in (offset / second)
def findObject():
    offset_h     = 0
    obj_velocity = 0
    
    # find object
    # find horizontal position relative to frame center
    # find obj velocity

    # ... INSERT CODE HERE ...
    
    return offset_h, obj_velocity

# MOVE TURRET HORIZONTAL
# Description: 
#   Moves the turret horizontally by activating the horizontal position motors.
#   If unable to achieve the desired movement, the furthest obtainable offset 
#   will be used.
# Inputs:
#   h_pos    : the current horizontal position of the turret
#   offset_h : the desired change in position of the turret
#   min_h_pos: the minimum valid horizontal position of the turret
#   max_h_pos: the maximum valid horizontal position of the turret
# Outputs:
#   new_h_pos: the new horizontal position of the turret
#   failed   : a boolean indicating that the desired offset was obtained
def moveTurretHorizontal(h_pos, offset_h, min_h_pos=MIN_H_POS, max_h_pos=MAX_H_POS):
    
    # move turret horizontally

    # ... INSERT CODE HERE ...
    
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
#   will be used.
# Inputs:
#   v_pos    : the current vertical position of the turret
#   offset_v : the desired change in position of the turret
#   min_v_pos: the minimum valid vertical position of the turret
#   max_v_pos: the maximum valid vertical position of the turret
# Outputs:
#   new_v_pos: the new vertical position of the turret
#   failed   : a boolean indicating that the desired offset was obtained
def moveTurretVertical(v_pos, offset_v, min_v_pos=MIN_V_POS, max_v_pos=MAX_V_POS):
    
    # move turret vertically

    # ... INSERT CODE HERE ...
    
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
#   max_range: the maximum range the range finder can obtain or the maximum
#       range the turret can target
# Outputs:
#   distance: range to the target (0 to MAX_RANGE, -1 if no valid target in range)
def getRange(max_range=MAX_RANGE):
    distance = -1
    
    # find the distance to the target
    # return -1 if no valid distance or out of range

    # ... INSERT CODE HERE ...
    
    if (distance > max_range):
        distance = -1
        
    return distance

# COMPUTE TRAJECTORY ANGLE
# Description:
#   Using the current position and distance to target, compute the needed vertical
#   offset in order to hit the target
# Inputs:
#   v_pos: current vertical position
#   distance: distance to target
# Outputs:
#   offset_v: the needed change in vertical position
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
#   activate: boolean; True to activate, False to deactivate
# Outputs:
#   is_active: the current state of the launcher
def activateTurret(activate):
    is_active = False
    
    # ... INSERT CODE HERE ...

    return is_active

# FIRE
# Description:
#   Activates the firing mechanism of the turret if the launcher has been activated
# Inputs:
#   is_turret_active: a boolean indicating if the launcher has been turned on. 
#       (See activateTurret)
# Outputs: None
def fire(is_turret_active):
    
    if (is_turret_active == True):
        is_turret_active == True
        # ... INSERT CODE HERE ...

    return