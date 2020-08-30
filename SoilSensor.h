/*
  SoilSensor.h - Library for handling Capacitive Soil Sensor
  Created by Pierre M., August 2020
*/
#ifndef SoilSensor_h
#define SoilSensor_h


class SoilSensor {
    private:
      String name;
      byte voltagePin;
      byte dataPin;
      int airValue;
      int waterValue;

    public:
      SoilSensor(String name, byte voltagePin, byte dataPin, int airValue, int waterValue) {
        this->name = name;
        this->voltagePin = voltagePin;
        this->dataPin = dataPin;
        this->airValue = airValue;
        this->waterValue = waterValue;
        init();
      }
      
      void init() {
        pinMode(voltagePin, OUTPUT);
      }
    
      void enable() {
         digitalWrite(this->voltagePin, HIGH);
      }
    
      void disable() {
         digitalWrite(voltagePin, LOW);
      }

      String getName() {
        return name;
      }
        
    
      char* getMoistureSensorValue(char* payload) {
        unsigned int soilMoistureValue = analogRead(dataPin);
        String message = this->name + ": Moisture reading: ";
        String result = message  + soilMoistureValue;
        unsigned int soilMoisturePercent = map(soilMoistureValue, airValue, waterValue, 0, 100);
        
        sprintf(payload, "%s", ""); // Cleans the payload content
        sprintf(payload, "{\"raw\": %s", String(soilMoistureValue)); // Adds the value
        sprintf(payload, "%s, \"pc\": %s}", payload, String(soilMoisturePercent)); // Adds percent
        
        return payload;  
      }
  
};

#endif
