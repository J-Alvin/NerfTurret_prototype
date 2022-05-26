import multiprocessing as mp

# Stores and Grabs Sensor Data
# Temperature is STored in Celcius
# Humidity is stored as a percentage in the form 50% = 0.50

AVERAGE_ROOM_TEMP = 20 # Celcius
AVERAGE_HUMIDITY = 0.5 # 50%

class HumidityTemperatureSensor:
    
    _temp = 0
    _humidity = 0
    
    def __init__(self, pins) -> None:
        print("Humidity and Temperature Sensor Controller: Initializing")
        self.pins = pins
        
        self._temp     = AVERAGE_ROOM_TEMP
        self._humidity = AVERAGE_HUMIDITY
        
        # initialize hardware

        # ... INSERT CODE HERE ...

        return
            
    def update(self):
        # updates member variables for temp and humidity
 
        # ... INSERT CODE HERE ...
        
        # placeholder
        self._temp     = AVERAGE_ROOM_TEMP
        self._humidity = AVERAGE_HUMIDITY
        
        return
    
    def getTemperatureCelcius(self):
        return self._temp
 
    def getHumidityPercentage(self):
        return self._humidity
    