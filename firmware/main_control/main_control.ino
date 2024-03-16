#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_I2CDevice.h>

Adafruit_I2CDevice Relay(0x0A);

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

void setup() {
  Serial.begin(9600);
  Serial.println("IrriGator");
 
#if defined (__XTENSA__)
  // On ESP-01: GPIO0(SDA), GPIO2(SCL)
  Wire.begin(0, 2);
#elif defined (__AVR__)
  // On Arduino Nano: A4(SDA), A5(SCL)
  Wire.begin();
#endif
  Serial.println("[ OK ] I2C");

  if(Relay.begin()) {
    Serial.print("[ OK ]");
  } else {
    Serial.print("[FAIL]");
  }
  Serial.println(" Relay controller");
}

void loop() {
  delay(1000);
  relayStatus();
  relaySet(true, 1);
  delay(1000);
  relayStatus();
  relaySet(true, 2);
  delay(1000);
  relayStatus();
  relaySet(false, 1);
  delay(1000);
  relayStatus();
  relaySet(false, 2);
}
