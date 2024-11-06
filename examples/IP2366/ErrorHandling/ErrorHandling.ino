#include <Wire.h>
#define INT_PIN 2  // Change this to your desired pin

#include "IP2366.h"

IP2366 device;

void setup() {
  Serial.begin(9600);
  device.begin();
  pinMode(INT_PIN, OUTPUT);
}

void loop() {
  digitalWrite(INT_PIN, HIGH); // Keep awake
  delay(110);
  
  uint8_t errorCode = 6;
  uint16_t VBATVoltage = device.getVBATVoltage(&errorCode);

  if (errorCode == 0) {
    Serial.print("Battery Voltage [mV]: ");
    Serial.println(VBATVoltage);
  } else {
    switch (errorCode) { //https://docs.arduino.cc/language-reference/en/functions/communication/Wire/endTransmission/
      case 1:
          Serial.println("data too long to fit in transmit buffer");
          break;
      case 2:
          Serial.println("received NACK on transmit of address");
          break;
      case 3:
          Serial.println("received NACK on transmit of data");
          break;
      case 4:
          Serial.println("other error");
          break;
      case 5:
          Serial.println("timeout");
          break;
      default:
          Serial.println("unknown error code");
          break;
}

  }


  delay(5000);
}
