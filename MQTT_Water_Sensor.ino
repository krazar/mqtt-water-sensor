//OTA include
#include "OTA.h"
#include "SoilSensor.h"
#include <PubSubClient.h>
#include <NewPing.h>

// ESP device: 192.168.2.32
const char* espName = "ESP32-1";


#define TIME_TO_SLEEP  15
#define TIME_TO_SLEEP_LONG  60

// Define Trig and Echo pin:
#define voltagePin 13
#define trigPin 14
#define echoPin 12
#define MAX_DISTANCE 180

// Moisture setup
const int AirValue = 3607;
const int WaterValue = 1280;  

String ota = "/ota";
String wildcard = "/#";
String otaTopic = espName + ota;
String espTopic = espName + wildcard;

uint8_t MQTT_MAX_RETRIES = 3;
uint8_t mqtt_failures = 0;

//boot counter for potential shutdowns
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int savedReading = 0;
touch_pad_t touchPin;

//sleeping options between reads
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define S_TO_M_FACTOR 60


// OTA topic
unsigned long entry;
bool waitingForUpdate = false;
long lastOtaWindow = 0;
bool noMessageRecievedYet = true;

//sonar object to deal with the bloody ultrasonic stuff

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

#define SENSOR_SIZE 1
SoilSensor sensors[]  = {SoilSensor("SoilSensor-1", 32, 33, 5000, 1226)};

//wifi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);
long duration = 0;
long startTime = 0;
bool debugConnection = false;

void setup() {

  Serial.begin(115200);
  
  //delay(1000);
  Serial.println("Booting");
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(voltagePin, OUTPUT);
  
  ++bootCount;
  noMessageRecievedYet = true;

  Serial.println("Woke up #: " + String(bootCount));
  digitalWrite(voltagePin, HIGH);
  
  client.setServer(mqttHost, mqttPort);
  client.setCallback(message_callback);
  //time to sleep
  
  int reading = measureData();

  digitalWrite(voltagePin, LOW);
  
  
  if((reading && savedReading != reading) || bootCount % 4 == 0) {
    savedReading = reading;
    startTime = millis();

    
    if(!setupOTA(espName)) {
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_LONG * uS_TO_S_FACTOR * S_TO_M_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP_LONG) + " Minutes");  
    } else {
      duration = millis() - startTime;
    
      // configure time to sleep
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * S_TO_M_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Minutes");  

       for(int i = 0; i<SENSOR_SIZE; i++){
         sensors[i].enable();
      }
    
      measureAndPublishData(reading);

      for(int j = 0; j<SENSOR_SIZE; j++){
      sensors[j].disable();
     }
     
      myLoop();  
      
    }
  
  
  } else {
    Serial.println("Don't send reading or no reading");
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * S_TO_M_FACTOR);
     Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Minutes");  
  }

  
  Serial.println("Entering deep sleep");
  Serial.flush(); 
  delay(100);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


bool send_MQTT_message(const char* message, const char* topic) {
  client.publish(topic, message, true);  
  return true;
}

void message_callback(char* topic, byte* payload, unsigned int length) {

  Serial.println("****************************** incoming message");
  if(String(topic) == otaTopic) {
    int otaEnabled = (int) payload[0] - '0';
    Serial.print("ota status: ");
    Serial.println(otaEnabled);
    switchOta(otaEnabled);
    noMessageRecievedYet = false;
  }
}

boolean reconnect() {
  // Loop until we're reconnected
  while (!client.connected() && mqtt_failures < MQTT_MAX_RETRIES) {
    ArduinoOTA.handle();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(espName, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(espTopic.c_str());
      return true;
    } else {
      mqtt_failures++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}

void switchOta(int state) {
  if(state == 1) {
    waitingForUpdate = true;
  } else {
    waitingForUpdate = false;
  }
}

bool isOtaWindow(){
  long now = millis();
  bool otaActivation = ( bootCount % 2 == 0) && now < (3000 + (100 * wifi_failures ) + 1000 * (mqtt_failures));
  if (otaActivation ) {
    lastOtaWindow = now;
  }
  bool result = otaActivation && noMessageRecievedYet;
  if (result) {
    Serial.println("-- In OTA Window --");
  }
  return result;
}

void loop() {}

void myLoop() {
 while(waitingForUpdate || isOtaWindow()) {

#ifndef ESP32_RTOS
ArduinoOTA.handle();
#endif

  if (!client.connected()) {
    
  }
  client.loop();
  delay(200);  
  }
 }

 int measureData() {
  
  Serial.println("Getting a reading");
  unsigned int reading = sonar.convert_cm(sonar.ping_median(5)) - 20;

  if (reading) {
    Serial.println("Reading succesfull: " + String(reading));
  }
    return reading;
  }

void measureAndPublishData(int reading) {
  if (!client.connected() && !reconnect()){
    return;
  }
  
  if (reading) {
    Serial.println("Publishing reading");
    if(reading > 180) {
      send_MQTT_message(String(reading).c_str(), "sensor/water_tank/bad");
    } else {
      send_MQTT_message(String(reading).c_str(), "sensor/water_tank");  
    }
  }

  if(debugConnection) {
    String debug = "/debug";
    String tN = espName + debug;
    char tC[20]; 
    tN.toCharArray(tC, 20);
    send_MQTT_message(String(duration).c_str(), tC);   
    send_MQTT_message(String(startTime).c_str(), tC);   
    send_MQTT_message(String(millis()).c_str(), tC);   
  }

  for(int i = 0; i<SENSOR_SIZE; i++){
    char result[100];
    sensors[i].getMoistureSensorValue(result);
    Serial.println(String(result));
    String topicName = "sensor/" + sensors[i].getName();
    char topicChar[20]; 
    topicName.toCharArray(topicChar, 20);
    send_MQTT_message(result, topicChar);
  }
  Serial.println("MQTT message published successfully");        
  
}
    
    
