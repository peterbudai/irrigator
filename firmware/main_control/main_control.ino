
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

#define BOOL2STR(b, s) ((b) ? (s) : "FAIL")
#define BOOL2OK(b) BOOL2STR(b, " OK ")

const char ssid[] = BLYNK_WIFI_SSID;
const char pass[] = BLYNK_WIFI_PASS;
const char auth[] = BLYNK_AUTH_TOKEN;

class RelayController {
private:
  static const uint8_t RELAY_1 = 0x01;
  static const uint8_t RELAY_2 = 0x02;
  static const uint8_t RELAY_OFF = 0x10;
  static const uint8_t RELAY_ON = 0x20;

  Adafruit_I2CDevice i2c;

public:
  RelayController()
    : i2c(0x0A) {
  }

  void begin() {
    bool ok = i2c.begin();
    Serial.printf("[%s] Relay controller init\n", BOOL2OK(ok));
  }

  void setState(bool on) {
    uint8_t cmd = (on ? RELAY_ON : RELAY_OFF) | RELAY_1 | RELAY_2;
    uint8_t state = 0;
    bool ok = i2c.write_then_read(&cmd, 1, &state, 1);
    Serial.printf("[%s] Relay change 0x%02x -> 0x%02x\n", BOOL2OK(ok), cmd, state);
  }
};


class UserController {
private:
  static const uint8_t USER_LED_OFF = 0x00;
  static const uint8_t USER_LED1_BLIP = 0x01;
  static const uint8_t USER_LED1_BLINK = 0x02;
  static const uint8_t USER_LED1_ON = 0x03;
  static const uint8_t USER_LED2_BLIP = 0x04;
  static const uint8_t USER_LED2_BLINK = 0x08;
  static const uint8_t USER_LED2_ON = 0x0C;
  static const uint8_t USER_SET_LED1 = 0x10;
  static const uint8_t USER_SET_LED2 = 0x20;
  static const uint8_t USER_ACK_BTN = 0x80;

  Adafruit_I2CDevice i2c;

  void setState(uint8_t cmd) {
    uint8_t state = 0;
    bool ok = i2c.write_then_read(&cmd, 1, &state, 1);
    Serial.printf("[%s] User interface change: 0x%02x -> 0x%02x\n", BOOL2OK(ok), cmd, state);
  }

public:
  UserController()
    : i2c(0x0B) {
  }

  void begin() {
    bool ok = i2c.begin();
    Serial.printf("[%s] User controller init\n", BOOL2OK(ok));
    this->setState(USER_ACK_BTN | USER_SET_LED1 | USER_SET_LED2 | USER_LED1_ON);
  }

  bool checkButton() {
    uint8_t state = 0;
    bool ok = i2c.read(&state, 1);
    Serial.printf("[%s] User interface status: 0x%02x\f", BOOL2STR(ok, " IN "), state);

    bool pressed = (state & USER_ACK_BTN) != 0;
    if(pressed) {
      this->setState(USER_ACK_BTN);
    }
    return pressed;
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
    bool ok = rtc.begin();
    Serial.printf("[%s] Clock init\n", BOOL2OK(ok));
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
      Serial.printf("[%s] Alarm %d set\n", BOOL2OK(success), alarm);
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
  Serial.println("[INFO] Cloud connected");

  // Request up-to-date settings from cloud
  Blynk.syncAll();                      // Schedule settings
  Blynk.sendInternal("rtc", "sync");    // Current time

  // Update user interface status LED
  User.setConnectionState(true);
}

// Handle successful connection to cloud server
BLYNK_DISCONNECTED() {
  Serial.println("[INFO] Cloud disconnected");

  // Update user interface status LED
  User.setConnectionState(false);
}

// Periodically check connection state
void checkConnection() {
  // Update user interface status LED
  // User.setConnectionState((WiFi.status() == WL_CONNECTED) && Blynk.connected());
  User.checkButton();
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

  wl_status_t wstatus = WiFi.begin(ssid, pass);
  Serial.printf("[%s] WiFi init", BOOL2OK(wstatus != WL_CONNECT_FAILED));

  Timer.setInterval(1000L, checkConnection);
  Timer.setInterval(60000L, measureTemperature);
  Blynk.config(auth);
  Serial.println("[ OK ] Blynk init");

  delay(1000);
  User.setConnectionState(false);
}

void loop() {
  Blynk.run();
  Timer.run();
}
