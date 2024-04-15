#include <Wire.h>
#define INT_PIN D4  // Change this to your desired pin

#include "IP2368.h"

IP2368 device;

void setup() {
  Serial.begin(9600);
  device.begin();
  pinMode(INT_PIN, OUTPUT);
}

void loop() {
  digitalWrite(INT_PIN, HIGH); // Keep awake
  delay(110);

  Serial.print("Battery Percentage [%]: ");
  Serial.println(device.getBatteryPercentage());
  Serial.print("Battery Voltage [mV]: ");
  Serial.println(device.getVBATVoltage());
  Serial.println(device.isCharging());
  Serial.println(device.isDischarging());

  IP2368::ChargeState currentState = device.getChargeState();
  Serial.println(device.isPDCharging());
  
  switch (currentState) {
    case IP2368::STANDBY:
        Serial.println("Standby");
        break;

    case IP2368::TRICKLE_CHARGE:
        Serial.println("Trickle Charge");
        break;

    case IP2368::CONSTANT_CURRENT:
        Serial.println("Constant Current Charging");
        break;

    case IP2368::CONSTANT_VOLTAGE:
        Serial.println("Constant Voltage Charging");
        break;

    case IP2368::CHARGE_WAIT:
        Serial.println("Charge Waiting (not started or other situations)");
        break;

    case IP2368::CHARGE_FULL:
        Serial.println("Charge Full");
        break;

    case IP2368::CHARGE_TIMEOUT:
        Serial.println("Charge Timeout");
        break;

    default:
        Serial.println("Unknown Charge State");
        break;
  }

  delay(5000);
}
