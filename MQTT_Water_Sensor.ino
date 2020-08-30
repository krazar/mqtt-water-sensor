//OTA include
#include "OTA.h"

#include <PubSubClient.h>
#include <NewPing.h>

// ESP device: 192.168.2.10
const char* espName = "ESP32-Dev-1";


#define TIME_TO_SLEEP  1
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

//boot counter for potential shutdowns
RTC_DATA_ATTR int bootCount = 0;
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

//wifi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);
long duration = 0;

void setup() {

  Serial.begin(115200);
  
  delay(1000);
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
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * S_TO_M_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Minutes");

  int reading = measureData();
  measureAndPublishData(reading);
  myLoop();

  digitalWrite(voltagePin, LOW);
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    ArduinoOTA.handle();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(espName, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(espTopic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
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
  bool otaActivation = ( bootCount % 2 == 0) && now < (5000 + (100 * wifi_failures ));
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
    reconnect();
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

  if (reading) {
    Serial.println("Publishing reading");
  }
}
    
    
