import multiprocessing as mp

CAMERA_CONTROL_PHASE_A = 2
CAMERA_CONTROL_PHASE_B = 3

class CameraController:
    def __init__(self, phase, target_found, desired_yaw, desired_pitch) -> None:
        print("Camera Controller: Initializing")
        self.phase = phase
        self.target_found = target_found    # used to indicate a valid target has been identitifed (1)
                                            # or lost/not found (0)
        self.desired_yaw = desired_yaw
        self.desired_pitch = desired_pitch


    # initial algorithm for process loop
    def start(self):

        # ... INSERT CODE HERE ...

        pass

    # loop cycle (single loop)
    def update(self):
      
        # control phase A
        p = self.phase.value
        if (p == CAMERA_CONTROL_PHASE_A):
            # control pitch and yaw to center turret
        
            # ... INSERT CODE HERE ...
            
            pass
        # control phase B
        elif (p == CAMERA_CONTROL_PHASE_B):
            # control yaw to lead turret
            
            # ... INSERT CODE HERE ...
            
            pass
        
        # check if target is still valid
        # set target found to False if target is lost
        self.target_found.value = 0 # no target
        
        return
    
    # life cycle of controller
    def run(self):
        self.start()
        while(True):
            self.update()
        return