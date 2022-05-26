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

#RUN_TEST = False
RUN_TEST = True

import math # For calculating the needed projectile trajectory
import time # For leading the target
import threading

# DESIGN ASSUMPTIONS:
#   -Camera, Range Finder, and Barrel move synchronously and inline with the turret
#   -Turret only targets objects in the Horizontal
#   -Vertical and horizontal positions measured in degrees
#   -Negative offsets move turret to the right/down, positive to the left/up
#   -v_pos = 0 results in the turret being completely level

# CHANGE LOG:
# Early Nov
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
# 30 Nov 2021
#   - changed to parallel programming
# 1 Dec 2021
#   - changed thread timings and fixed shared resource bug in Thread 3

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

# SHARED RESOURCES
# All are single reads/writes. No need to protect with mutex

# Threads 1, 2, 3
# run_threads: programing is running/not
run_threads = True

# Threads 1, 3  
#   target_found:    has the camera found a target
#       Thread 1 writes
#       Thread 3 reads
target_found = False

# Threads 2, 3
#   v_offset:   vertical offset necessary to reach target
#       Thread 2 writes
#       Thread 3 reads
#   v_pos:      current vertical position of the turret
#       Thread 1 reads
#       Thread 3 writes
v_offset = 0
v_pos = 0

# END SHARED RESOURCES

# The Forever Loop
def loop():  
    
    # CREATE THREADS
    
    # Timings
    # timing3       > timing1                   > timing2
    # fire delay    > find target/move_h delay  > get Range delay
    
    # create thread to get camera data and move turret horizontally
    camera          = initCamera()          # findObject()
    horizontalMotor = initHorizontalMotor() # moveTurretHorizontal()
    timing1         = 50                    # how often do we check for targets 
                                            # and move turret horizontally
    thread1         = threading.Thread(target=thread1_func, 
                                       args=(camera, horizontalMotor, timing1))
    
    # create thread to get range finder data
    rangeFinder     = initRangeFinder()     # getRange()
    timing2         = 5                     # how often do we get range data
                                            # can be zero, a timing > 0 reduces computations
    thread2         = threading.Thread(target=thread2_func, 
                                       args=(rangeFinder, timing2))
    
    # set up main thread
    verticalMotor   = initVerticalMotor()   # moveTurretVertical()
    armingHardware  = initArmingHardware()  # activateTurret()
    triggerHardware = initTriggerHardware() # fire()
    timing3         = 100                   # how often do we fire (max rate of fire)
    thread3         = threading.Thread(target=thread3_func, 
                                       args=(verticalMotor, armingHardware, triggerHardware, timing3))
    
    # start threads
    thread1.start()
    thread2.start()
    thread3.start()
    
    # Manual termination of threads for testing
    time.sleep(2)
    global run_threads
    run_threads = False
    
    return

# THREAD 1 FUNC
# Description:
#   Thread inifintely looping. Thread finds a target in the camera, moves the 
#   turret horizontally, and calculates the velocity of the target
# Input:
#   camera:             camera hardware interface
#   horizonatalMotor:   horizontal motor interface
#   delay:              delay in millisecs for each thread loop (0 to disable). 
#                       Delay gives other processes time to register that a target is found
# Shared Resources:
#   target_found:    has the camera found a target (write)
def thread1_func(camera, horizontalMotor, delay=100):
 
    # adjust delay( milli-sec -> sec)
    if (delay > 0):
        delay /= 1000

    # set access to shared resource
    global run_threads 
    
    global target_found
    
    
    # initialize
    h_pos = 0
    
    while(1):
        if (run_threads == False):
            return
        
        if (delay > 0):
            time.sleep(delay)
        
        target_found = False
        
        # find target
        # compute offset
        offset_h, obj_velocity = findObject(camera)
        
        # Center Turret on Target: Horizontally
        h_pos, h_failed = moveTurretHorizontal(horizontalMotor, h_pos, offset_h)
        
        
        # If unable to complete horizontal movement, object is to far out of 
        # frame or out of maximum horizontal movement. Therefore, restart loop
        if (h_failed):
            continue
        else:
            target_found = True
        
        ## IGNORE: adjust velocity, lead target
        
        # continue loop
        
    return

# THREAD 2 FUNC
# Description:
#   Thread infinittely looping. Thread constantely retreives range finder data
#   and computes the vertical offset from the given distance.
# Inputs:
#   rangeFinder: range finder hardware interface
#   delay: delay in milli-secs for each loop (0 to disable)
# Shared Resources:
#   v_offset:   vertical offset necessary to reach target (write)
#   v_pos:      current vertical position of the turret (read)
def thread2_func(rangeFinder, delay=0):
    # adjust delay( milli-sec -> sec)
    if (delay > 0):
        delay /= 1000

    # set access to shared resource
    global run_threads 
    
    global offset_v 
    
    # initialize
    distance = -1
    
    while(1):
        if (run_threads == False):
            return
        
        # ? delay
        if (delay > 0):
            time.sleep(delay)
        
        # Get Distance 
        distance = getRange(rangeFinder)
        
        # If no target within distance, restart loop
        if (distance == -1):
            continue
        
        # Compute Vertical Offset from Distance to Target
        offset_v = computeTrajectoryAngle(v_pos, distance)
        
        # continue loop
        
    return

# THREAD 3 FUNC
# Description:
#   Thread inifinitely looping. Thread constantely checks if a target is found.
#   It then moves the turret vertically and fires.
# Inputs:
#   verticalMotor:      vertical morotor hardware interafce
#   armingHardware:     arming hardware interface
#   triggerHardware:    trigger hardware interface
#   delay:              delay in milli-secs for task loop (0 to disable)
# Shared Resources:
#   v_offset:   vertical offset of turret needed to reach target (read)
#   v_pos:      current vertical position of the turret (write)
# 
#   target_found:   indicates target is found (read)
def thread3_func(verticalMotor, armingHardware, triggerHardware, delay=100):
    
    # set access to shared resources
    global run_threads 
    
    global target_found
    
    global v_pos
    global offset_v
        
    # adjust delay( milli-sec -> sec)
    if (delay > 0):
        delay /= 1000
    
    # initialize
    is_launcher_active = False
    v_failed = True
    
    while(1):
        # disarm turret
        is_launcher_active = False
        is_launcher_active = activateTurret(armingHardware, False, is_launcher_active)
    
        if (run_threads == False):
            return
        
        # ? delay
        if (delay > 0):
            time.sleep(delay)
            
        if (target_found == True):
            # grab vertical offset
            offset_v_local  = offset_v
            
            # arm turret
            is_launcher_active = activateTurret(armingHardware, True, is_launcher_active)
            
            # move turret vertically
            v_pos, v_failed = moveTurretVertical(verticalMotor, v_pos, offset_v_local)
            if (v_failed):
                continue
            
            # fire
            fire(triggerHardware, is_launcher_active)
            
            # move turret back to level
            v_pos, v_failed = moveTurretVertical(verticalMotor, v_pos, -offset_v_local)
            
        # continue loop
        
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