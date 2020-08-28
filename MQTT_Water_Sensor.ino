//OTA include
#include "OTA.h"

#include <PubSubClient.h>
#include <NewPing.h>

// ESP device: 192.168.2.10
const char* espName = "ESP32-Dev-1";


#define TIME_TO_SLEEP  15

// Define Trig and Echo pin:
#define voltagePin 13
#define trigPin 14
#define echoPin 12
#define MAX_DISTANCE 180

// Moisture setup
const int AirValue = 3607;
const int WaterValue = 1280;  
#define soilPin1 35
#define voltageSoilPin1 34


String ota = "/ota";
String wildcard = "/#";
String otaTopic = espName + ota;
String espTopic = espName + wildcard;

/* switch mqtt for hassio to enable ota mode
 * - platform: mqttf
  name: "esp32-dev1"
  command_topic: "ESP32-Dev-1/ota"
  optimistic: true
  payload_on: "1"
  payload_off: "0"
  retain: true
 */


//boot counter for potential shutdowns
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

//sleeping options between reads
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define S_TO_M_FACTOR 60


// wifi and mqtt reconnect
uint8_t wifi_failures = 0;
uint8_t mqtt_failures = 0;


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
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {

  Serial.begin(115200);
  //wait for serial, remove prod
  //delay(1000);
  Serial.println("Booting");
  
  //setupOTA(espName);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(voltagePin, OUTPUT);
  pinMode(voltageSoilPin1, OUTPUT);

  wifi_failures = 0;
  mqtt_failures = 0;
  
  ++bootCount;
  noMessageRecievedYet = true;

  Serial.println("Woke up #: " + String(bootCount));

  digitalWrite(voltagePin, HIGH);
  digitalWrite(voltageSoilPin1, HIGH);
  //setup_wifi();

  
  client.setServer(mqttHost, mqttPort);
  client.setCallback(message_callback);
  //time to sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * S_TO_M_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Minutes");

  measureAndPublishData();
  myLoop();

  digitalWrite(voltagePin, LOW);
  digitalWrite(voltageSoilPin1, LOW);
  
  Serial.println("Entering deep sleep");
  Serial.flush(); 
  delay(100);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

bool setup_wifi() {
  delay(10);
  
  Serial.print("Connecting to ");
  Serial.println(mySSID);

  WiFi.begin(mySSID, myPASSWORD);
  
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


char* getMoistureSensorValue(const int pin) {
  char payload[100];
  unsigned int soilMoistureValue = analogRead(pin);
  String message = "Moisture reading: ";
  String result = message  + soilMoistureValue;
  Serial.println(result);
  unsigned int soilMoisturePercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  
  sprintf(payload, "%s", ""); // Cleans the payload content
  sprintf(payload, "{\"raw\": %s", String(soilMoistureValue)); // Adds the value
  sprintf(payload, "%s, \"pc\": \"%s\"}", payload, String(soilMoisturePercent)); // Adds percent
  return payload;
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
      mqtt_failures++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
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
  bool otaActivation = ( bootCount % 2 == 0) && now < (5000 + (5000 *(wifi_failures + mqtt_failures)));
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

 void measureAndPublishData() {


  long now = millis();
  
  Serial.println("Getting a reading");
  unsigned int reading = sonar.convert_cm(sonar.ping_median(5)) - 20;

  // char* moisture1 = getMoistureSensorValue(soilPin1);
  if (reading) {
    Serial.println("Reading succesfull: " + String(reading));
  }
    Serial.println("Writting on MQTT");
delay(5000);
  }
    
