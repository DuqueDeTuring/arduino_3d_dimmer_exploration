/*
  Proof of concept taking code from the Arduino Examples:

  - ArduinoMqttClient - WiFi Simple Sender
  - Arduino LSM6DS3 - Simple Gyroscope
  - Arduino LSM6DS3 - Simple Accelerometer

  --------------

  Probando una idea tomando código de los ejemplos de Arduino:

  - ArduinoMqttClient - WiFi Simple Sender
  - Arduino LSM6DS3 - Simple Gyroscope
  - Arduino LSM6DS3 - Simple Accelerometer

*/

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
#include <ESP8266WiFi.h>
#endif

#include "arduino_secrets.h"
#include <Arduino_LSM6DS3.h>

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = SECRET_MQTT_BROKER_IP;
int        port     = SECRET_MQTT_BROKER_PORT;
const char topic[]  = "arduino/accelerator";

const long interval = 100;
unsigned long previousMillis = 0;

bool switchLocked = true;
const long lockInterval = 2000;
unsigned long lockMillis = 0;
const int lockSwitchAngle = 80;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    halt();
  }
 
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  signalReady();
}

void halt() {
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

// just to indicate system readiness, we blink the led a few times with increased frequency
void signalReady() {
  for (int countdown = 1000; countdown > 50;) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(countdown);
    digitalWrite(LED_BUILTIN, LOW);
    countdown = countdown / 2;
    delay(countdown);
  }

}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  // if we are in the period lockInterval we ignore the sensors
  if (lockMillis > 0 && (currentMillis - lockMillis >= lockInterval)) {
    lockMillis = 0;

  } else if (lockMillis > 0) {
    return;
  }

  // using the gyroscope
  //  if (!switchLocked) {
  //    float x, y, z;
  //    if (IMU.gyroscopeAvailable()) {
  //      IMU.readGyroscope(x, y, z);
  //      //    Serial.print(x);
  //      //    Serial.print('\t');
  //      //    Serial.println(y);
  //      //    Serial.print('\t');
  //      //    Serial.println(z);
  //    }
  //
  //    if (currentMillis - previousMillis >= interval) {
  //      previousMillis = currentMillis;
  //      if (y < -50) {
  //        mqttClient.beginMessage(topic);
  //        mqttClient.print("D");
  //        mqttClient.endMessage();
  //      }
  //
  //      if (y >= 50) {
  //        mqttClient.beginMessage(topic);
  //        mqttClient.print("U");
  //        mqttClient.endMessage();
  //
  //      }
  //    }
  //  }


  if (lockMillis == 0 && IMU.accelerationAvailable()) {
    int degreesX, degreesY = 0;

    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    if (ax > 0.1) {
      ax = 100 * ax;
      degreesX = map(ax, 0, 97, 0, 90);
      Serial.print("DOWN:");
      Serial.println(degreesX);
      if (!switchLocked) {
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          if (degreesX > 20) {
            mqttClient.beginMessage(topic);
            mqttClient.print("D");
            mqttClient.endMessage();
          }
        }
      }
    }
    if (ax < -0.1) {
      ax =  100 * ax;
      degreesX = map(ax, 0, -100, 0, 90);
//      Serial.print("UP:");
//      Serial.println(degreesX);
      if (!switchLocked) {
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          if (degreesX > 20) {
            mqttClient.beginMessage(topic);
            mqttClient.print("U");
            mqttClient.endMessage();
          }
        }
      }

    }

    if (ay > 0.1) {
      ay = 100 * ay;
      degreesY = map(ay, 0, 97, 0, 90);
//      Serial.print("LEFT:");
//      Serial.println(degreesY);
      if (degreesY > lockSwitchAngle) {
        switchLocked = false;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    } else if (ay < -0.1) {
      ay =  100 * ay;
      degreesY = map(ay, 0, -100, 0, 90);
//      Serial.print("RIGHT:");
//      Serial.println(degreesY);
      if (!switchLocked) {
        if (degreesY > lockSwitchAngle) {
          switchLocked = true;
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
    }
  
  }
}
