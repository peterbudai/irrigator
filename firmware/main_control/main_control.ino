
#define BLYNK_NO_FANCY_LOGO
// #define BLYNK_DEBUG
#define BLYNK_PRINT Serial

#include "secrets.h"      // Must be first, before any Blynk-related includes

#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_I2CDevice.h>
#include <RTClib.h>

const char ssid[] = BLYNK_WIFI_SSID;
const char pass[] = BLYNK_WIFI_PASS;
const char auth[] = BLYNK_AUTH_TOKEN;

Adafruit_I2CDevice Relay(0x0A);
Adafruit_I2CDevice User(0x0B);
RTC_DS3231 Clock;
BlynkTimer Timer;

void relaySet(bool on, uint8_t num) {
  uint8_t cmd = (on ? 0x20 : 0x10) | (num & 0x03);
  if(!Relay.write(&cmd, 1)) {
    Serial.println("Communication error with relay");
  }
}

void relayStatus() {
  uint8_t status = 0;
  Relay.read(&status, 1);
  Serial.print("[0x");
  Serial.print(status, 16);
  Serial.println("] Relay status");
}

void userSet(bool ack, uint8_t leds) {
  uint8_t cmd = (ack ? 0xB0 : 0x30) | (leds & 0x0F);
  if(!User.write(&cmd, 1)) {
    Serial.println("Communication error with user");
  }
}

uint8_t userStatus() {
  uint8_t status = 0;
  User.read(&status, 1);
  Serial.print("[0x");
  Serial.print(status, 16);
  Serial.println("] User status");
  return status;
}

void checkConnection() {
  // State.setBlink((millis() % 1500) < 750);
  // State.setConnection(WiFi.status() == WL_CONNECTED ? (Blynk.connected() ? CONN_SERVER : CONN_WIFI) : CONN_NONE);
}


void setup() {
  Serial.begin(9600);
  Serial.println("IrriGator");
 
  // SDA on GPIO0, SCL on GPIO2
  Wire.begin(0, 2);
  Serial.println("[ OK ] I2C");

  if(User.begin()) {
    Serial.print("[ OK ]");
  } else {
    Serial.print("[FAIL]");
  }
  Serial.println(" User controller");

  if(Relay.begin()) {
    Serial.print("[ OK ]");
  } else {
    Serial.print("[FAIL]");
  }
  Serial.println(" Relay controller");


  WiFi.begin(ssid, pass);

  Timer.setInterval(250L, checkConnection);
  
  Blynk.config(auth);

  delay(1000);
}

void loop() {
  Blynk.run();
  Timer.run();
}
