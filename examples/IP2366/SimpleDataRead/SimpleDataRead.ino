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

  Serial.print("Battery Voltage [mV]: ");
  Serial.println(device.getVBATVoltage());

  Serial.print("Is Charging?: ");
  Serial.println(device.isCharging());

  Serial.print("Is Fast Charge?: ");
  Serial.println(device.isFastCharge());

  Serial.print("Is DisCharging?: ");
  Serial.println(device.isDischarging());

  IP2366::ChargeState currentState = device.getChargeState();
  switch (currentState) {
    case IP2366::ChargeState::STANDBY:
        Serial.println("Standby");
        break;

    case IP2366::ChargeState::TRICKLE_CHARGE:
        Serial.println("Trickle Charge");
        break;

    case IP2366::ChargeState::CONSTANT_CURRENT:
        Serial.println("Constant Current Charging");
        break;

    case IP2366::ChargeState::CONSTANT_VOLTAGE:
        Serial.println("Constant Voltage Charging");
        break;

    case IP2366::ChargeState::CHARGE_WAIT:
        Serial.println("Charge Waiting (not started or other situations)");
        break;

    case IP2366::ChargeState::CHARGE_FULL:
        Serial.println("Charge Full");
        break;

    case IP2366::ChargeState::CHARGE_TIMEOUT:
        Serial.println("Charge Timeout");
        break;

    default:
        Serial.println("Unknown Charge State");
        break;
  }

  delay(5000);
}
