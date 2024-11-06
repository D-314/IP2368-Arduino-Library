#include <Arduino.h>
#include <Wire.h>
#include "IP2366.h"
#include <cstring>

IP2366 chip; // Initialize the chip with default I2C address

void setup() {
  Serial.begin(115200); // Start the serial communication

  Wire.setClock(100000);

  chip.begin();       // Initialize the chip

  delay(100);

  //Optionally, configure the chip as needed
  chip.ResetMCU(true);
  delay(100);
  chip.enableLoadOTP(false);
  chip.enableCharger(true);
  delay(100);
  chip.setMaxInputPowerOrBatteryCurrent(5000); //5A
  chip.setTypeCMode(IP2366::TypeCMode::UFP);
}




// Helper functions to convert enums to strings

const char* toString(IP2366::ChargeState state) {
  switch (state) {
    case IP2366::ChargeState::STANDBY: return "Standby";
    case IP2366::ChargeState::TRICKLE_CHARGE: return "Trickle Charge";
    case IP2366::ChargeState::CONSTANT_CURRENT: return "Constant Current";
    case IP2366::ChargeState::CONSTANT_VOLTAGE: return "Constant Voltage";
    case IP2366::ChargeState::CHARGE_WAIT: return "Charge Wait";
    case IP2366::ChargeState::CHARGE_FULL: return "Charge Full";
    case IP2366::ChargeState::CHARGE_TIMEOUT: return "Charge Timeout";
    default: return "Unknown";
  }
}

const char* toString(IP2366::TypeCMode mode) {
  switch (mode) {
    case IP2366::TypeCMode::UFP: return "UFP (Consumer)";
    case IP2366::TypeCMode::DFP: return "DFP (Power Source)";
    case IP2366::TypeCMode::DRP: return "DRP (Dual Role)";
    default: return "Unknown";
  }
}

void loop() {
  char buffer[2048]; // Буфер для форматированной строки

  // Заполнение буфера данными
  snprintf(buffer, sizeof(buffer), 
           "System Status:\n"
           "Load One Time Programmable (OTP) Enabled: %d\n"
           "Charger Enabled: %d\n"
           "Current Setting Mode: %d (mA)",
           chip.isLoadOTPEnabled(),
           chip.isChargerEnabled(),
           chip.getMaxInputPowerOrBatteryCurrent());

  // Добавление остальных данных в буфер
  snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
           "Standby Mode Enabled: %d\n"
           "DC-DC Output Enabled: %d\n"
           "Type-C Mode: %s\n",
           chip.isStandbyModeEnabled(),
           chip.isDcDcOutputEnabled(),
           toString(chip.getTypeCMode()));

  // Добавление данных о зарядке в буфер
  snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
           "\nCharging Status:\n"
           "Is Charging: %d\n"
           "Is Charge Full: %d\n"
           "Is Discharging: %d\n"
           "Charge State: %s\n"
           "Charge Voltage: %.2f V\n",
           chip.isCharging(),
           chip.isChargeFull(),
           chip.isDischarging(),
           toString(chip.getChargeState()),
           (float)chip.getChargeVoltage() / 1000);

  // Добавление информации о батарее в буфер
  snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
           "\nBattery Info:\n"
           "Battery Voltage: %.2f V\n"
           "Battery Current: %.2f A\n",
           (float)chip.getVBATVoltage() / 1000,
           (float)chip.getBATCurrent() / 1000);

  // Добавление информации о Vsys в буфер
  snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
           "\nVsys Info:\n"
           "Vsys Voltage: %.2f V\n"
           "Vsys Current: %.2f A\n"
           "Vsys Power: %.2f W\n",
           (float)chip.getVsysVoltage() / 1000,
           (float)chip.getVsysCurrent() / 1000,
           (float)chip.getVsysPower() / 1000);


  snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
          "--------------------------------------------------------------------");
  // Вывод буфера
  Serial.println(buffer);

  delay(1000); // Задержка перед повторением цикла
}
