
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

  void setState(uint8_t state) {
    if(i2c.write(&state, 1)) {
      Serial.printf("[0x%02x]", state);
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" User control");
  }

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
    this->setState(USER_ACK_BTN | USER_SET_LED1 | USER_SET_LED2 | USER_LED1_ON);
  }

  bool checkButton() {
    uint8_t state = 0;
    if(i2c.read(&state, 1)) {
      Serial.printf("[0x%02x]", state);
    } else {
      Serial.print("[FAIL]");
    }
    Serial.println(" User status");
    return (state & USER_ACK_BTN) != 0;
  }

  void setConnectionState(bool connected) {
    if(connected) {
      this->setState(USER_SET_LED1 | USER_LED1_BLIP);
    } else {
      this->setState(USER_SET_LED1 | USER_LED1_BLINK);
    }
  }
};

class ClockController {
private:
  bool enabled;
  bool timeValid;
  bool alarmValid[2];

  RTC_DS3231 rtc;

public:
  ClockController()
    : enabled(false), timeValid(false), alarmValid({false, false}) {
  }

  void begin() {
    Serial.printf("[%s] Clock init\n", rtc.begin() ? " OK " : "FAIL");
  }

  void setTime(const DateTime& dt) {
    rtc.adjust(dt);
    timeValid = true;
    Serial.printf("[ OK ] Current time set\n");  
  }

  void setAlarm(uint8_t alarm, const TimeInputParam& input) {
    if(alarm < 1 || alarm > 2) {
      return;
    }

    if(input.hasStartTime()) {
      DateTime time(0, 0, 0, input.getStartHour(), input.getStartMinute(), input.getStartSecond());
      bool success = false;
      if(alarm == 1) {
        success = rtc.setAlarm1(time, Ds3231Alarm1Mode::DS3231_A1_Hour);
      }
      if(alarm == 2) {
        success = rtc.setAlarm2(time, Ds3231Alarm2Mode::DS3231_A2_Hour);
      }
      alarmValid[alarm - 1] = true;
      Serial.printf("[%s] Alarm %d set\n", success ? " OK " : "FAIL", alarm);
    } else {
      rtc.disableAlarm(alarm);
      alarmValid[alarm - 1] = false;
      Serial.printf("[ OK ] Alarm %d disable\n", alarm);
    }
  }

  float getTemperature() {
    float temp = rtc.getTemperature();
    Serial.printf("[ IN ] Temperature: %.2f C\n", temp);
    return temp;
  }
};

#define VirtualPinTimeOn V0
#define VirtualPinTimeOff V1
#define VirtualPinPinSchedule V2
#define VirtualPinStatus V3
#define VirtualPinTemperature V4

RelayController Relay;
UserController User;
ClockController Clock;
BlynkTimer Timer;

// Handle receiving fresh real-time clock value from cloud
BLYNK_WRITE(InternalPinRTC) {
  // Convert Unix timestamp to date/time for RTC
  DateTime time(param.asLong());
  String str = time.timestamp();
  Serial.printf("[RECV] Current time: %s\n", str.c_str());  
  Clock.setTime(time);
}

// Handle receiving new schedule ON time from cloud
BLYNK_WRITE(VirtualPinTimeOn) {
  TimeInputParam input(param);
  Serial.printf("[RECV] On time: %02d:%02d\n", input.getStartHour(), input.getStartMinute());
  Clock.setAlarm(1, input);
}

// Handle receiving new schedule OFF time from cloud
BLYNK_WRITE(VirtualPinTimeOff) {
  TimeInputParam input(param);
  Serial.printf("[RECV] Off time: %02d:%02d\n", input.getStartHour(), input.getStartMinute());  
  Clock.setAlarm(2, input);
}

// Handle successful connection to cloud server
BLYNK_CONNECTED() {
  Serial.println("[ OK ] Cloud connected");

  // Request up-to-date settings from cloud
  Blynk.syncAll();                      // Schedule settings
  Blynk.sendInternal("rtc", "sync");    // Current time

  // Update user interface status LED
  User.setConnectionState(true);
}

// Handle successful connection to cloud server
BLYNK_DISCONNECTED() {
  Serial.println("[FAIL] Cloud disconnected");

  // Update user interface status LED
  User.setConnectionState(false);
}

// Periodically check connection state
void checkConnection() {
  // Update user interface status LED
  if(User.checkButton()) {
    User.setConnectionState((WiFi.status() == WL_CONNECTED) && Blynk.connected());
  }
}

void measureTemperature() {
  Blynk.virtualWrite(VirtualPinTemperature, Clock.getTemperature()), 
  Serial.println("[SEND] Temperature");
}

void setup() {
  Serial.begin(9600);
  Serial.println("IrriGator");
 
  // SDA on GPIO0, SCL on GPIO2
  Wire.begin(0, 2);
  Serial.println("[ OK ] I2C init");

  User.begin();
  Relay.begin();
  Clock.begin();

  if(WiFi.begin(ssid, pass) == WL_CONNECT_FAILED) {
    Serial.println("[FAIL] WiFi init");
  } else {
    Serial.println("[ OK ] WiFi init");
  }

  Timer.setInterval(1000L, checkConnection);
  Timer.setInterval(60000L, measureTemperature);
  Blynk.config(auth);
  Serial.println("[ OK ] Cloud init");

  delay(1000);
  User.setConnectionState(false);
}

void loop() {
  Blynk.run();
  Timer.run();
}
