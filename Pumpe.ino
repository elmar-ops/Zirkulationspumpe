#include <math.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#define WIFI_SSID "Hasenhausen24G"
#define WIFI_PASSWORD "racketomauz"

#define SENSOR_PIN             A0
#define REFERENCE_RESISTANCE   10000
#define NOMINAL_RESISTANCE     10000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

#define RELAIS D0            // GPIO4 230VAC-Relais

// Raspberri Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 178, 118)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

#define MQTT_PUB_TEMP "esp/pump/temperature"
#define MQTT_PUB_INFO "esp/pump/info"

const int hysterese_cel = 2;
const int runtimer_min = 1; //4
const int lock_time_min = 2; //60
const int force_pump_hour = 8; // pump every 8h!

unsigned long prevTempMillis = 0;           // previous temperature measurement timer
unsigned long prevZirkMillis = 0;           // previous runtimer measurement timer
unsigned long prevLockMillis = 0;           // previous Locking measurement timer
unsigned long prevForceMillis = 0;          // previous force measurement timer
unsigned long prevMinuteMillis = 0;         // previous count minutes timer
unsigned long prevUACMillis = 0;            // previous analog input reading timer

double celsius = 0;
double prev_celsius = 0;

bool zirk = false;
bool locked = false;

Thermistor* thermistor;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

const int ledPin =  13;      // the number of the LED pin

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  //Serial.print("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D0, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  thermistor = new NTC_Thermistor(
    SENSOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE
  );

  prevTempMillis = millis();
  prevZirkMillis = millis();
  prevLockMillis = millis();
  prevForceMillis = millis();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  
  connectToWifi();

  digitalWrite(RELAIS, LOW);
  digitalWrite(LED_BUILTIN, HIGH);  
}

// the loop function runs over and over again forever
void loop() {
  unsigned long currentMillis = millis();

  // force every X h!
  if (currentMillis - prevForceMillis >= force_pump_hour * 1000 * 3600) {   
    Serial.println("FORCE circulation!");
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "FORCE circulation!"); 
    zirk = true;  
    prevZirkMillis = currentMillis;  
    prevForceMillis = currentMillis;
  }

  if (currentMillis - prevTempMillis >= 10000){   // Take a measurement at every 10 sec
    celsius = thermistor->readCelsius();
    if (prev_celsius + hysterese_cel <= celsius) {
      if (locked == false) {
         Serial.println("Turn on circulation");
         uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "Turn on circulation"); 
         zirk = true;  
         prevZirkMillis = currentMillis;
      } else {
        Serial.println("Pump still locked");
        uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "Pump still locked"); }
    }

    prevTempMillis = currentMillis;  //Remember the last time measurement
    Serial.print("Temperature: ");
    Serial.println(celsius);
     // Publish an MQTT message on topic esp/dht/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(celsius).c_str());                            
    //Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    //Serial.printf("Message: %.2f \n", celsius);
    prev_celsius = celsius;
  } 
  
  // disable lock
  if ((currentMillis - prevLockMillis >= lock_time_min * 1000 * 60) and locked == true){   
    Serial.println("Turn off lock");  
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "Turn off lock"); 
    locked == false;
  }

  if (zirk == true ) {
    digitalWrite(RELAIS, HIGH);
    digitalWrite(LED_BUILTIN, LOW); 
    if (currentMillis - prevZirkMillis >= runtimer_min * 1000 * 60) { // auto aus 
      prevZirkMillis = currentMillis;
      zirk = false;
      digitalWrite(RELAIS, LOW);
      digitalWrite(LED_BUILTIN, HIGH);  
      Serial.println("Turn off circulation");  
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "Turn off circulation"); 
      Serial.println("lock pump");  
      packetIdPub1 = mqttClient.publish(MQTT_PUB_INFO, 1, true, "lock pump"); // lock pump for lock_time_min
      locked = true;
      prevLockMillis = currentMillis;
    }
  } 

 
}
