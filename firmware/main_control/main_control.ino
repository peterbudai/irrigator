#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_I2CDevice.h>

Adafruit_I2CDevice Relay(0x0A);
Adafruit_I2CDevice User(0x0B);

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



void setup() {
  Serial.begin(9600);
  Serial.println("IrriGator");
 
  // GPIO0(SDA), GPIO2(SCL)
  Wire.begin(0, 2);
  Serial.println("[ OK ] I2C");

  if(Relay.begin()) {
    Serial.print("[ OK ]");
  } else {
    Serial.print("[FAIL]");
  }
  Serial.println(" Relay controller");

  if(User.begin()) {
    Serial.print("[ OK ]");
  } else {
    Serial.print("[FAIL]");
  }
  Serial.println(" User controller");
}

void loop() {
  uint8_t us = 0;

  delay(1000);
  relayStatus();
  relaySet(true, 1);
  us = userStatus();
  if(us & 0x80) {
    us = (us + 1) & 0x0F;
    userSet(true, us);
  }
  delay(1000);
  relayStatus();
  relaySet(true, 2);
  us = userStatus();
  if(us & 0x80) {
    us = (us + 1) & 0x0F;
    userSet(true, us);
  }
  delay(1000);
  relayStatus();
  relaySet(false, 1);
  us = userStatus();
  if(us & 0x80) {
    us = (us + 1) & 0x0F;
    userSet(true, us);
  }
  delay(1000);
  relayStatus();
  relaySet(false, 2);
  us = userStatus();
  if(us & 0x80) {
    us = (us + 1) & 0x0F;
    userSet(true, us);
  }
}
