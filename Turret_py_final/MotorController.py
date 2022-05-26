"""
    Handles a single stepper motor on the pins passed to it
"""
import multiprocessing as mp    

class MotorController:
    def __init__(self, pins, desired_rotation) -> None:
        assert len(pins) == 4, "MotorController: Incorrect number of pins"

        self.desired_rotation = desired_rotation
        self.MotorInterfaceType  = 8
        # self.stepper = AccelStepper(MotorInterfaceType, pin[0], pint[1], pin[2], pin[3]);
        # self.stepped.setMaxSpeed(500)
        print("Motor Controller: Initializing on pins: {pins}")

        
    # initial algorithm for process loop
    def start(self):
        
        # ... INSERT CODE HERE ...
        
        pass
    
    # loop cycle (single loop)
    def update(self):
        
        # ... INSERT CODE HERE ...

        # adjust motor according to desired rotation
        rot = self.desired_rotation.value
        # stepper.setSpeed(1000);
        # stepper.runSpeed();
        pass
    
    # life cycle of controller
    def run(self):
        self.start()
        while(True):
            self.update()
        return