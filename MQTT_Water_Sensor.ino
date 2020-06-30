//OTA include
#include "OTA.h"

#include <PubSubClient.h>
#include <NewPing.h>

unsigned long entry;

// ESP device
const char* espName = "ESP32-Dev-1";

//sleeping options between reads
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5

//boot counter for potential shutdowns
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

// Define Trig and Echo pin:
#define trigPin 14
#define echoPin 5
#define MAX_DISTANCE 400

// Moisture setup
const int AirValue = 3607;   //you need to replace this value with Value_1
const int WaterValue = 1280;
#define soilPin1 35

// OTA topic
bool waitingForUpdate = false;

long lastOtaWindow = 0;
String ota = "/ota";
String wildcard = "/#";
String otaTopic = espName + ota;
String espTopic = espName + wildcard;

//sonar object to deal with the bloody ultrasonic stuff
NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

//wifi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Booting");

  setupOTA(espName);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //SERIAL CRAP, remove in prod
  delay(500);
  Serial.begin(9600);
  ++bootCount;

  Serial.println("Woke up #: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  print_wakeup_touchpad();

  setup_wifi();
  client.setServer(mqttHost, mqttPort);
  client.setCallback(message_callback);
  //time to sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

bool setup_wifi() {
  delay(10);
  
  Serial.print("Connecting to ");
  Serial.println(mySSID);

  WiFi.begin(mySSID, myPASSWORD);

  uint8_t wifi_failures=0;
  
  while (WiFi.status() != WL_CONNECTED || wifi_failures > 10) {
    ArduinoOTA.handle();
    delay(500);
    Serial.print(".");
    wifi_failures++;
  }
  if(wifi_failures > 10){
    return false;
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

bool send_MQTT_message(const char* message, const char* topic) {
  client.publish(topic, message);  
  return true;
}

void message_callback(char* topic, byte* payload, unsigned int length) {

  Serial.println("****************************** incoming message");
  if(String(topic) == otaTopic) {
    int otaEnabled = (int) payload[0] - '0';
    Serial.print("ota status: ");
    Serial.println(otaEnabled);
    if(otaEnabled == 1) {
      enableOta();
    } else {
      disableOta();
      
    }
  }
}


int getMoistureSensorValue(const int pin) {
  unsigned int soilMoistureValue = analogRead(pin);
  String message = "Moisture reading: ";
  String result = message  + soilMoistureValue;
  Serial.println(result);
  unsigned int soilMoisturePercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  return map(soilMoistureValue, AirValue, WaterValue, 0, 100);
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
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void enableOta() {
  if(!waitingForUpdate) {
    Serial.println("enabling ota");
    waitingForUpdate = true;
  }
}

void disableOta() {
  if(waitingForUpdate) {
    Serial.println("disabling ota");
    waitingForUpdate = false;
  }
}

bool isOtaWindow(){
  long now = millis();
  bool pendingOtaWindow =  (lastOtaWindow + 1000) > now;
  bool otaActivation = ( bootCount % 3 == 0) && now < 10000;
  if (otaActivation ) {
    lastOtaWindow = now;
  }
  bool result = otaActivation || pendingOtaWindow;
  if (result) {
    Serial.println("-- In OTA Window --");
  }
  return result;
}

void loop() {

#ifndef ESP32_RTOS
ArduinoOTA.handle();
#endif
 
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    Serial.println("Getting a reading");
    unsigned int reading = sonar.convert_cm(sonar.ping_median(5));
  
    int moisture1 = getMoistureSensorValue(soilPin1);
    
    if (reading) {
      Serial.println("Reading succesfull");
     
      Serial.println("Writting on MQTT");
      if(
        send_MQTT_message(String(reading).c_str(), "sensor/water_tank") &&
        send_MQTT_message(String(moisture1).c_str(), "sensor/soil_moisture_1") &&
        send_MQTT_message(String(moisture1).c_str(), "coucou")
        ) {
            Serial.println("MQTT message published successfully");
            
        } else {
            Serial.println("MQTT Issue. Trying later");
        }
      }
    }
  
    if(waitingForUpdate || isOtaWindow()) {
       delay(50);  
       } else {
      Serial.println("Entering deep sleep");
      delay(200);
      esp_deep_sleep_start();
    
    }

}

//Function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

/*
Method to print the touchpad by which ESP32
has been awaken from sleep
*/
void print_wakeup_touchpad(){
  touch_pad_t pin;

  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch(touchPin)
  {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 0"); break;
    case 2  : Serial.println("Touch detected on GPIO 2"); break;
    case 3  : Serial.println("Touch detected on GPIO 15"); break;
    case 4  : Serial.println("Touch detected on GPIO 13"); break;
    case 5  : Serial.println("Touch detected on GPIO 12"); break;
    case 6  : Serial.println("Touch detected on GPIO 14"); break;
    case 7  : Serial.println("Touch detected on GPIO 27"); break;
    case 8  : Serial.println("Touch detected on GPIO 33"); break;
    case 9  : Serial.println("Touch detected on GPIO 32"); break;
    default : Serial.print("Wakeup not by touchpad "); Serial.print(touchPin);break;
  }
}
