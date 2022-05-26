import multiprocessing as mp
import HumidityTemperatureSensor
import time # get the sleep function

DISTANCE_CONTROL_PHASE = 3

AVERAGE_SOUND_SPEED = 343 # meters/sec

AVERAGE_AIR_PRESSURE = 84732.53322 # Pa
# p = 101325 (1 - ( (2.25577*10^-5) * h) )^5.25588
# p = air pressure (Pa)
# h = elevation (meters)
# elevation of Rexburg Idaho: 1483 m

class DistanceSensorController:
    def __init__(self, phase, desired_pitch, humid_temp_sensor) -> None:
        print("Distance Sensor Controller: Initializing")
        self.phase = phase
        self.desired_pitch = desired_pitch
        self.humid_temp_sensor = humid_temp_sensor
        self.soundSpeed = AVERAGE_SOUND_SPEED
        self.air_pressure = AVERAGE_AIR_PRESSURE
        return

    # initial algorithm for process loop
    def start(self):
        
        # update sensor data
        self.humid_temp_sensor.update()

        # get initial readings
        temp = self.humid_temp_sensor.getTemperatureCelcius()
        humidity = self.humid_temp_sensor.getHumidityPercentage()
        
        # compute speed of sound through air using
        # temp, humidity, and air pressure

        #   ... INSERT CODE HERE ... 
        # default
        self.soundSpeed = AVERAGE_SOUND_SPEED
        
        return
    
    # loop cycle (single loop)
    def update(self):
        # If we are in control, adjust pitch according to distance
        
        p = self.phase.value
        if (p == DISTANCE_CONTROL_PHASE):
            
            # get distance
            
            #   ... INSERT CODE HERE ...

            # compute pitch

            #   ... INSERT CODE HERE ...
            
            # adjust pitch
            
            #   ... INSERT CODE HERE ...

            pass
        
        return
    
    # life cycle of controller
    def run(self):
        self.start()
        while(True):
            self.update()
        return
    
    def compute():
        pass
        # signal = pin 8
        # receive = pin 9
        # signal = High
        # sleep(.000010) # 10 microseconds
        # signal = Low
        # startTime = perf_counter()
        # while(receive == High)
        #   pass
        # endTime = perf_counter()
        # elapsedTime = endTime - startTime
        # distance = elapsedTime*speedOfSound()
        # sleep(.002) #measure every 2 milliseconds
        # #speed of sound
        # import Adafruit_DHT #Need to find library and download
        # sensor = Adafruit_DHT.DHT22
        # pin = 23
        # humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
        # velocity = 331+(.6*temperature)
        # return velocity
