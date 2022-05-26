"""
Main
Entry point for the program and hadnles the general state and flow.
"""

# Multiproccessing
import multiprocessing as mp
import time

# Hardware Controllers
import CameraController
import MotorController
import DistanceSensorController
import HumidityTemperatureSensor

# Pins for the stepper motors
YAW_MOTOR_PINS    = (0, 0, 0, 0)
PITCH_MOTOR_PINS  = (0, 0, 0, 0)
FIRING_MOTOR_PINS = (0, 0, 0, 0)
HUMID_TEMP_PINS   = (0, 0, 0, 0)

LAUNCHER_PIN = 0

# Constants
FIRE_DURATION   = 2.0   # Duration of each burst in seconds
FIRE_PROJECTILE = 1     # Value to fire projectile

class Manager:
    
    # Initialize our Manager
    def __init__( self ):
        print("Manager: Initializing.")

        # Multiprocessing Values
        # aka Shared Memory
        self.phase = mp.Value("i", 0)
        self.target_found = mp.Value("i", 0)
        self.desired_yaw_rotation = mp.Value("i", 0)
        self.desired_pitch_rotation = mp.Value("i", 0)
        self.should_fire = mp.Value("i", 0)

        # Make our motors.
        self.yaw_motor = MotorController.MotorController( YAW_MOTOR_PINS, self.desired_yaw_rotation )
        self.pitch_motor = MotorController.MotorController( PITCH_MOTOR_PINS, self.desired_pitch_rotation )
        self.firing_motor = MotorController.MotorController( FIRING_MOTOR_PINS, self.should_fire )

        # Make our camera.
        self.cam_controller = CameraController.CameraController(self.phase, self.target_found, self.desired_yaw_rotation, self.desired_pitch_rotation )


        # Make our distance sensor controller.
        # Make our humidity and temperature sensor to give it
        self.humid_temp_sensor = HumidityTemperatureSensor.HumidityTemperatureSensor(HUMID_TEMP_PINS)
        self.dist_controller = DistanceSensorController.DistanceSensorController(self.phase, self.desired_pitch_rotation, self.humid_temp_sensor)

        # process storage
        self._processes = []
        return
            
    def start(self):        
        # Initialize
        
        # ... INSERT CODE HERE ...
        
        # Create Threads
        # Pitch, Yaw, RangeFinder, Camera, Firing Motor
        self._processes = []
        self._processes.append(mp.Process(target=self.pitch_motor.run))
        self._processes.append(mp.Process(target=self.yaw_motor.run))
        self._processes.append(mp.Process(target=self.firing_motor.run))
        self._processes.append(mp.Process(target=self.cam_controller.run))
        self._processes.append(mp.Process(target=self.dist_controller.run))

        # start each process
        for p in self._processes:
            p.start()
            
        
        # Start Phase 1.
        self.start_phase_one()
        
        return
        
    def update( self ):
        
        # main loop
        p = self.phase.value
        if (p == 1):
            self.evaluate_phase_one()            
        elif (p == 2):
            self.evaluate_phase_two()
        elif (p == 3):
            self.evaluate_phase_three()
        elif (p == 4):
            self.evaluate_phase_four()
        else:
            ## error occurred
            # reset phase to phase 1
            self.start_phase_one()
            
        # check for lost target
        # reset to phase 1 if target is lost
        target_found = self.target_found.value
        if (p > 1 and target_found == 0):
            print("Manager: Target Lost.")
            self.start_phase_one()

        return

    # Clean Up ??
    def end(self):
        for p in self.processes:
            p.kill()
        return
    
    """
    Phase One
        Idle, Awaiting Target
    """
    def start_phase_one( self ):
        print("Manager: Starting Phase One.")
        self.phase.value = 1
        
        return

    def evaluate_phase_one( self ):

        # check if camera has found target
        target_found = self.target_found.value
        if (target_found == 1):
            self.start_phase_two()
            
        return

    """
    Phase Two
        Target Found, Center in frame
    """
    def start_phase_two( self ):
        print("Manager: Starting Phase Two.")
        self.phase.value = 2
        # camera now takes control of motors
        # CameraController sets desired_pitch_rotation, desired_yaw_rotation
        return

    def evaluate_phase_two( self ):
        
        # wait for turret to be centered on target
  
        yaw = self.desired_yaw_rotation.value
        pitch = self.desired_pitch_rotation.value
        
        # if yaw and pitch don't need to be changed (i.e. == 0), we are centered
        if (yaw == 0 and pitch == 0):
            self.start_phase_three()
            
        return

    """
    Phase Three
        Adjust for Speed and Distance
    """
    def start_phase_three( self ):
        print("Manager: Starting Phase Three.")
        self.phase.value = 3
        # distance controller and camera take control of motors
        # DistanceSensorControler sets desired_pitch_rotation
        # CameraController sets desired_yaw_rotatio for leading
        return

    def evaluate_phase_three( self ):
        
        # wait for turret to stop adjusting for distance and leading

        yaw = self.desired_yaw_rotation.value
        pitch = self.desired_pitch_rotation.value
        
        # if yaw and pitch don't need to be changed (i.e. == 0), we are centered
        if (yaw == 0 and pitch == 0):
            self.start_phase_four()
            
        return

    """
    Phase Four
    Fire 
    """

    def start_phase_four( self ):
        print("Manager: Starting Phase Four.")
        
        self.phase.value = 4
        
        return

    def evaluate_phase_four( self ):
        
        # turn on trigger for a certain amount of time
        self.should_fire.value = FIRE_PROJECTILE

        # wait
        time.sleep(FIRE_DURATION)
        
        # turn off trigger
        self.should_fire.value = 0

        # then go back to phase 1
        self.start_phase_one()

        return        


if __name__ == "__main__":
    manager = Manager()
    manager.start()

    # Loop the program
    while(True):
        manager.update()
        
    manager.end()