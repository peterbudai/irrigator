
#define BLYNK_FIRMWARE_VERSION "0.1.0"
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

#define RELAY_1 0x01
#define RELAY_2 0x02
#define RELAY_OFF 0x10
#define RELAY_ON 0x20

class RelayController {
private:
  Adafruit_I2CDevice i2c;

public:
  RelayController()
    : i2c(0x0A) {
  }

  void begin() {
    if(i2c.begin()) {
      Serial.print("[ OK ]");
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" Relay controller init");
  }

  void setState(bool on) {
    uint8_t cmd = (on ? RELAY_ON : RELAY_OFF) | RELAY_1 | RELAY_2;
    if(i2c.write(&cmd, 1)) {
      Serial.printf("[0x%02x]", cmd);
    } else {
      Serial.print("[FAIL]");
    }
      Serial.println(" Relay control");
  }

  void logState() {
    uint8_t state = 0;
    if(i2c.read(&state, 1)) {
      Serial.printf("[0x%02x]", state);
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" Relay status");
  }
};

#define USER_LED_OFF 0x00
#define USER_LED1_BLIP 0x01
#define USER_LED1_BLINK 0x02
#define USER_LED1_ON 0x03
#define USER_LED2_BLIP 0x04
#define USER_LED2_BLINK 0x08
#define USER_LED2_ON 0x0C
#define USER_SET_LED1 0x10
#define USER_SET_LED2 0x20
#define USER_ACK_BTN 0x80

class UserController {
private:
  Adafruit_I2CDevice i2c;

public:
  UserController()
    : i2c(0x0B) {
  }

  void begin() {
    if(i2c.begin()) {
      Serial.print("[ OK ]");
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" User controller init");
  }

  void setState(uint8_t state) {
    if(i2c.write(&state, 1)) {
      Serial.printf("[0x%02x]", state);
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" User control");
  }

  bool checkButton() {
    uint8_t state = 0;
    if(i2c.read(&state, 1)) {
      Serial.printf("[0x%02x]");
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" User status");
    return (state & USER_ACK_BTN) != 0;
  }
};

RelayController Relay;
UserController User;
RTC_DS3231 Clock;
BlynkTimer Timer;

void checkConnection() {
  // State.setBlink((millis() % 1500) < 750);
  // State.setConnection(WiFi.status() == WL_CONNECTED ? (Blynk.connected() ? CONN_SERVER : CONN_WIFI) : CONN_NONE);
}

BLYNK_WRITE(InternalPinRTC) {     //check the value of InternalPinRTC  
  DateTime now(param.asLong());   //store time in t variable
  Serial.print("Current time: ");  
  Serial.println(now.timestamp());
  //Clock.adjust(now);
}

BLYNK_CONNECTED() {
  Blynk.syncAll();
  Blynk.sendInternal("rtc", "sync"); //request current local time for device
}

void setup() {
  Serial.begin(9600);
  Serial.println("IrriGator");
 
  // SDA on GPIO0, SCL on GPIO2
  Wire.begin(0, 2);
  Serial.println("[ OK ] I2C init");

  User.begin();
  User.setState(USER_ACK_BTN | USER_SET_LED1 | USER_SET_LED2 | USER_LED1_ON);
  Relay.begin();

  if(WiFi.begin(ssid, pass) == WL_CONNECT_FAILED) {
    Serial.println("[FAIL] WiFi init");
  } else {
    Serial.println("[ OK ] WiFi init");
  }

  Timer.setInterval(250L, checkConnection);
  Blynk.config(auth);
  Serial.println("[ OK ] Blynk init");

  delay(1000);
}

void loop() {
  Blynk.run();
  Timer.run();
}
