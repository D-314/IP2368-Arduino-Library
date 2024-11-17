#include "IP2366.h"
#include <Arduino.h>

// Registers

// System Control Registers
#define IP2366_REG_SYS_CTL0 0x00  // Charge enable and other control settings
#define IP2366_REG_SYS_CTL2 0x02  // Vset full-charge voltage setting
#define IP2366_REG_SYS_CTL3 0x03  // Iset charge power or current setting
#define IP2366_REG_SYS_CTL4 0x04  // Battery capacity setting
#define IP2366_REG_SYS_CTL6 0x06  // Trickle charge current, threshold and charge timeout setting
#define IP2366_REG_SYS_CTL8 0x08  // Stop charge current and recharge threshold setting
#define IP2366_REG_SYS_CTL9 0x09  // Standby enable and low battery voltage settings
#define IP2366_REG_SYS_CTL10 0x0A // Low battery voltage setting
#define IP2366_REG_SYS_CTL11 0x0B // Output enable register
#define IP2366_REG_SYS_CTL12 0x0C // Output maximum power selection register

// TYPE-C Control Registers
#define IP2366_REG_SYS_CTL12 0x0C   // Output maximum power selection register
#define IP2366_REG_SELECT_PDO 0x0D  // select charging PDO gear
#define IP2366_REG_TypeC_CTL8 0x22  // TYPE-C mode control register
#define IP2366_REG_TypeC_CTL9 0x23  // Output Pdo current setting register
#define IP2366_REG_TypeC_CTL10 0x24 // 5VPdo current setting register
#define IP2366_REG_TypeC_CTL11 0x25 // 9VPdo current setting register
#define IP2366_REG_TypeC_CTL12 0x26 // 12VPdo current setting register
#define IP2366_REG_TypeC_CTL13 0x27 // 15VPdo current setting register
#define IP2366_REG_TypeC_CTL14 0x28 // 20VPdo current setting register
#define IP2366_REG_TypeC_CTL17 0x2B // Output Pdo setting register
#define IP2366_REG_TypeC_CTL23 0x29 // Pps1 Pdo current setting register
#define IP2366_REG_TypeC_CTL24 0x2A // Pps2 Pdo current setting register
#define IP2366_REG_TypeC_CTL18 0x2C // PDO plus 10mA current enable, needs to be configured together with the current setting register

// Read-only Status Indication Registers
#define IP2366_REG_STATE_CTL0 0x31   // Charge status control register
#define IP2366_REG_STATE_CTL1 0x32   // Charge status control register 2
#define IP2366_REG_STATE_CTL2 0x33   // Input Pd status control register
#define IP2366_REG_TypeC_STATE 0x34  // System status indication register
#define IP2366_REG_RECEIVED_PDO 0x35 // receive PDO gear
#define IP2366_REG_STATE_CTL3 0x38   // System over-current indication register

// ADC Data Registers
#define IP2366_REG_BATVADC_DAT0 0x50         // VBAT voltage low 8 bits
#define IP2366_REG_BATVADC_DAT1 0x51         // VBAT voltage high 8 bits
#define IP2366_REG_VsysVADC_DAT0 0x52        // Vsys voltage low 8 bits
#define IP2366_REG_VsysVADC_DAT1 0x53        // Vsys voltage high 8 bits
#define IP2366_REG_TIMENODE1 0x69            // 1st bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP2366_REG_TIMENODE2 0x6A            // 2nd bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP2366_REG_TIMENODE3 0x6B            // 3rd bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP2366_REG_TIMENODE4 0x6C            // 4th bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP2366_REG_TIMENODE5 0x6D            // 4th bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP2366_REG_IBATIADC_DAT0 0x6E        // BAT-end current low 8 bits
#define IP2366_REG_IBATIADC_DAT1 0x6F        // BAT-end current high 8 bits
#define IP2366_REG_ISYS_IADC_DAT0 0x70       // IVsys-end current low 8 bits
#define IP2366_REG_IVsys_IADC_DAT1 0x71      // IVsys-end current high 8 bits
#define IP2366_REG_Vsys_POW_DAT0 0x74        // Vsysterminal power low 8 bits
#define IP2366_REG_Vsys_POW_DAT1 0x75        // Vsysterminal power high 8 bits

// Additional ADC Data Registers for NTC, GPIOs
#define IP2366_REG_INTC_IADC_DAT0 0x77     // NTC output current setting
#define IP2366_REG_VGPIO0_NTC_DAT0 0x78    // VGPIO0_NTC ADC voltage low 8 bits
#define IP2366_REG_VGPIO0_NTC_DAT1 0x79    // VGPIO0_NTC ADC voltage high 8 bits

//#define ADC_TO_MV(adc_val) ((uint16_t)((((uint32_t)(adc_val) * 3300) / 0xFFFF)))

// SYS_CTL2

void IP2366::setFullChargeVoltage(uint16_t voltage, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint16_t minVoltage, maxVoltage;
    minVoltage = 2500;
    maxVoltage = 4400;

    if (voltage < minVoltage)
        voltage = minVoltage;
    else if (voltage > maxVoltage)
        voltage = maxVoltage;

    uint8_t regValue = (voltage - minVoltage) / 10;
    writeRegister(IP2366_REG_SYS_CTL2, regValue, errorCode);
}

uint16_t IP2366::getFullChargeVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue = readRegister(IP2366_REG_SYS_CTL2, errorCode);

    uint16_t baseVoltage = 2500;

    return baseVoltage + regValue * 10;
}

// SYS_CTL3

void IP2366::setMaxInputPowerOrBatteryCurrent(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue;
    uint16_t maxCurrent = 9700;
    if (current_mA > maxCurrent)
        current_mA = maxCurrent;

    regValue = current_mA / 100;

    writeRegister(IP2366_REG_SYS_CTL3, regValue, errorCode);
}

uint16_t IP2366::getMaxInputPowerOrBatteryCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue = readRegister(IP2366_REG_SYS_CTL3, errorCode);

    return regValue * 100;
}

// SYS_CTL6

void IP2366::setTrickleChargeCurrent(uint16_t current, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue = current / 50;
    writeRegister(IP2366_REG_SYS_CTL6, regValue, errorCode);
}

uint16_t IP2366::getTrickleChargeCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue = readRegister(IP2366_REG_SYS_CTL6, errorCode);
    return static_cast<uint16_t>(regValue * 50);
}


void IP2366::Standby(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP2366_REG_SYS_CTL9, errorCode);
    value = bitWrite(value, 6, enable);
    writeRegister(IP2366_REG_SYS_CTL9, value, errorCode);
}

bool IP2366::isStandby(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_SYS_CTL9, errorCode) & (1 << 6);
}

// SYS_CTL10

void IP2366::setLowBatteryVoltage(uint16_t voltage_mV, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t voltageSetting;
    uint16_t stepVoltage = 100;

    uint16_t baseVoltage = 2500;

    if (voltage_mV < baseVoltage)
    {
        voltage_mV = baseVoltage;
    }
    else if (voltage_mV > (baseVoltage + stepVoltage * 7))
    {
        voltage_mV = (baseVoltage + stepVoltage * 7);
    }

    voltageSetting = (voltage_mV - baseVoltage) / stepVoltage;

    uint8_t currentValue = readRegister(IP2366_REG_SYS_CTL10, errorCode) & 0x1F;
    writeRegister(IP2366_REG_SYS_CTL10, (voltageSetting << 5 | currentValue), errorCode);
}

uint16_t IP2366::getLowBatteryVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t voltageSetting = (readRegister(IP2366_REG_SYS_CTL10, errorCode) >> 5) & 0x03;

    uint16_t baseVoltage = 2500;
    return baseVoltage + voltageSetting * 100;
}

// SYS_CTL12

void IP2366::setMaxOutputPower(Vbus1OutputPower power, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP2366_REG_SYS_CTL12, errorCode) & 0x1F; 
    value |= static_cast<uint8_t>(power) << 5;                 
    writeRegister(IP2366_REG_SYS_CTL12, value, errorCode);
}

IP2366::Vbus1OutputPower IP2366::getMaxOutputPower(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return static_cast<Vbus1OutputPower>((readRegister(IP2366_REG_SYS_CTL12, errorCode) >> 5) & 0x07); 
}

// SELECT_PDO

void IP2366::setChargingPDOmode(ChargingPDOmode mode, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t currentValue = readRegister(IP2366_REG_SYS_CTL10, errorCode) & 0xF8;
    writeRegister(IP2366_REG_SELECT_PDO, (static_cast<uint8_t>(mode) | currentValue), errorCode);
}

IP2366::ChargingPDOmode IP2366::getChargingPDOmode(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return static_cast<ChargingPDOmode>(readRegister(IP2366_REG_SELECT_PDO, errorCode) & 0x03);
}

// TypeC_CTL18

void IP2366::enableSrcPdoAdd10mA(bool en5VPdoAdd10mA, bool en9VPdoAdd10mA, bool en12VPdoAdd10mA, bool en15VPdoAdd10mA, bool en20VPdoAdd10mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP2366_REG_TypeC_CTL18, errorCode);
    value = bitWrite(value, 4, en20VPdoAdd10mA);
    value = bitWrite(value, 3, en15VPdoAdd10mA);
    value = bitWrite(value, 2, en12VPdoAdd10mA);
    value = bitWrite(value, 1, en9VPdoAdd10mA);
    value = bitWrite(value, 0, en5VPdoAdd10mA);
    writeRegister(IP2366_REG_TypeC_CTL18, value, errorCode);
}

bool IP2366::isSrcPdoAdd10mA5VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_TypeC_CTL18, errorCode) & (1 << 0);
}

bool IP2366::isSrcPdoAdd10mA9VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_TypeC_CTL18, errorCode) & (1 << 1);
}

bool IP2366::isSrcPdoAdd10mA12VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_TypeC_CTL18, errorCode) & (1 << 2);
}

bool IP2366::isSrcPdoAdd10mA15VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_TypeC_CTL18, errorCode) & (1 << 3);
}

bool IP2366::isSrcPdoAdd10mA20VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_TypeC_CTL18, errorCode) & (1 << 4);
}

// RECEIVED_PDO

bool IP2366::isReceives5VPdo(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_RECEIVED_PDO, errorCode) & (1 << 0);
}

bool IP2366::isReceives9VPdo(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_RECEIVED_PDO, errorCode) & (1 << 1);
}

bool IP2366::isReceives12VPdo(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_RECEIVED_PDO, errorCode) & (1 << 2);
}

bool IP2366::isReceives15VPdo(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_RECEIVED_PDO, errorCode) & (1 << 3);
}

bool IP2366::isReceives20VPdo(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP2366_REG_RECEIVED_PDO, errorCode) & (1 << 4);
}

// TIMENODE

void IP2366::getTimenode(char timenode[5], uint8_t * errorCode) {
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    for (uint8_t i = 0; i < 5; i++)
    {
        timenode[i] = readRegister((IP2366_REG_TIMENODE1+i), errorCode);
    }
}

