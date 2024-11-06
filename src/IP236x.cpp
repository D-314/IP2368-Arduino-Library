#include "IP236x.h"
#include <Arduino.h>

// Registers

// System Control Registers
#define IP236x_REG_SYS_CTL0 0x00  // Charge enable and other control settings
#define IP236x_REG_SYS_CTL2 0x02  // Vset full-charge voltage setting
#define IP236x_REG_SYS_CTL3 0x03  // Iset charge power or current setting
#define IP236x_REG_SYS_CTL4 0x04  // Battery capacity setting
#define IP236x_REG_SYS_CTL6 0x06  // Trickle charge current, threshold and charge timeout setting
#define IP236x_REG_SYS_CTL8 0x08  // Stop charge current and recharge threshold setting
#define IP236x_REG_SYS_CTL9 0x09  // Standby enable and low battery voltage settings
#define IP236x_REG_SYS_CTL10 0x0A // Low battery voltage setting
#define IP236x_REG_SYS_CTL11 0x0B // Output enable register
#define IP236x_REG_SYS_CTL12 0x0C // Output maximum power selection register

// TYPE-C Control Registers
#define IP236x_REG_SYS_CTL12 0x0C   // Output maximum power selection register
#define IP236x_REG_SELECT_PDO 0x0D  // select charging PDO gear
#define IP236x_REG_TypeC_CTL8 0x22  // TYPE-C mode control register
#define IP236x_REG_TypeC_CTL9 0x23  // Output Pdo current setting register
#define IP236x_REG_TypeC_CTL10 0x24 // 5VPdo current setting register
#define IP236x_REG_TypeC_CTL11 0x25 // 9VPdo current setting register
#define IP236x_REG_TypeC_CTL12 0x26 // 12VPdo current setting register
#define IP236x_REG_TypeC_CTL13 0x27 // 15VPdo current setting register
#define IP236x_REG_TypeC_CTL14 0x28 // 20VPdo current setting register
#define IP236x_REG_TypeC_CTL17 0x2B // Output Pdo setting register
#define IP236x_REG_TypeC_CTL23 0x29 // Pps1 Pdo current setting register
#define IP236x_REG_TypeC_CTL24 0x2A // Pps2 Pdo current setting register
#define IP236x_REG_TypeC_CTL18 0x2C // PDO plus 10mA current enable, needs to be configured together with the current setting register

// Read-only Status Indication Registers
#define IP236x_REG_STATE_CTL0 0x31   // Charge status control register
#define IP236x_REG_STATE_CTL1 0x32   // Charge status control register 2
#define IP236x_REG_STATE_CTL2 0x33   // Input Pd status control register
#define IP236x_REG_TypeC_STATE 0x34  // System status indication register
#define IP236x_REG_RECEIVED_PDO 0x35 // receive PDO gear
#define IP236x_REG_STATE_CTL3 0x38   // System over-current indication register

// ADC Data Registers
#define IP236x_REG_BATVADC_DAT0 0x50         // VBAT voltage low 8 bits
#define IP236x_REG_BATVADC_DAT1 0x51         // VBAT voltage high 8 bits
#define IP236x_REG_VsysVADC_DAT0 0x52        // Vsys voltage low 8 bits
#define IP236x_REG_VsysVADC_DAT1 0x53        // Vsys voltage high 8 bits
#define IP236x_REG_TIMENODE1 0x69            // 1st bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP236x_REG_TIMENODE2 0x6A            // 2nd bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP236x_REG_TIMENODE3 0x6B            // 3rd bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP236x_REG_TIMENODE4 0x6C            // 4th bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP236x_REG_TIMENODE5 0x6D            // 4th bit of the timestamp register (the timestamp symbol is an ASCII character)
#define IP236x_REG_IBATIADC_DAT0 0x6E        // BAT-end current low 8 bits
#define IP236x_REG_IBATIADC_DAT1 0x6F        // BAT-end current high 8 bits
#define IP236x_REG_ISYS_IADC_DAT0 0x70       // IVsys-end current low 8 bits
#define IP236x_REG_IVsys_IADC_DAT1 0x71      // IVsys-end current high 8 bits
#define IP236x_REG_Vsys_POW_DAT0 0x74        // Vsysterminal power low 8 bits
#define IP236x_REG_Vsys_POW_DAT1 0x75        // Vsysterminal power high 8 bits

// Additional ADC Data Registers for NTC, GPIOs
#define IP236x_REG_INTC_IADC_DAT0 0x77     // NTC output current setting
#define IP236x_REG_VGPIO0_NTC_DAT0 0x78    // VGPIO0_NTC ADC voltage low 8 bits
#define IP236x_REG_VGPIO0_NTC_DAT1 0x79    // VGPIO0_NTC ADC voltage high 8 bits

#define ADC_TO_MV(adc_val) ((uint16_t)((((uint32_t)(adc_val) * 3300) / 0xFFFF)))

void IP236x::begin()
{
#ifdef TwoWire_h
    _wire->begin();
#endif
}

uint8_t IP236x::writeRegister(uint8_t regAddress, uint8_t value, uint8_t * errorCode)
{
#ifdef TwoWire_h
    _wire->beginTransmission(_address);
    delay (1);
    _wire->write(regAddress);
    delay (1);
    _wire->write(value);
    delay (1);
    uint8_t _errorCode = _wire->endTransmission(); // Send a stop signal
    delay (1);

    if (_errorCode)
    {
        if (errorCode != nullptr)
        {
            *errorCode = _errorCode; // write error code only if it > 0
        }
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

uint8_t IP236x::readRegister(uint8_t regAddress, uint8_t * errorCode)
{
#ifdef TwoWire_h
    _wire->beginTransmission(_address);
    delay (1); // increase the delay by 1ms between each byte
    _wire->write(regAddress);
    delay (1); // increase the delay by 1ms between each byte
    uint8_t _errorCode = _wire->endTransmission(false); // Do not send a stop signal
    delay (1);
    
    if (_errorCode)
    {
        if (errorCode != nullptr)
        {
            *errorCode = _errorCode; // write error code only if it > 0
        }
        return -1;
    }

    uint8_t bytesRead = _wire->requestFrom(_address, (uint8_t)1, (bool)true); // Request 1 byte and send a stop signal

    if (bytesRead != 1)
    {
        if (errorCode != nullptr)
        {
            *errorCode = _errorCode; // write error code only if it > 0
        }
        return 0;
    }
    return _wire->read(); //read from I2C internal buffer
#else
    return -1;
#endif
}

// SYS_CTL0

void IP236x::enableCharger(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 0, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isChargerEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x01;
}

void IP236x::enableVbusSinkSCP(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 2, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isVbusSinkSCPEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x04;
}

void IP236x::enableVbusSinkPD(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 3, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isVbusSinkPDEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x08;
}

void IP236x::enableVbusSinkDPdM(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 4, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isVbusSinkDPdMEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x10;
}

void IP236x::enableINTLow(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 5, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isINTLowEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x20;
}

void IP236x::ResetMCU(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 6, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

void IP236x::enableLoadOTP(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL0, errorCode);
    value = bitWrite(value, 7, enable);
    writeRegister(IP236x_REG_SYS_CTL0, value, errorCode);
}

bool IP236x::isLoadOTPEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL0, errorCode) & 0x80;
}

// SYS_CTL2

void IP236x::setFullChargeVoltage(uint16_t voltage, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
}

uint16_t IP236x::getFullChargeVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
    return 0;
}

// SYS_CTL3

void IP236x::setMaxInputPowerOrBatteryCurrent(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
}

uint16_t IP236x::getMaxInputPowerOrBatteryCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
    return 0;
}

// SYS_CTL6

void IP236x::setTrickleChargeCurrent(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
}

uint16_t IP236x::getTrickleChargeCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
    return 0;
}

// SYS_CTL8

void IP236x::setChargeStopCurrent(uint16_t current, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    if (current > 750)
        current = 750;
    uint8_t regValue = current / 50;

    uint8_t currentValue = readRegister(IP236x_REG_SYS_CTL8, errorCode) & 0x0F;
    writeRegister(IP236x_REG_SYS_CTL8, currentValue | (regValue << 4), errorCode);
}

uint16_t IP236x::getChargeStopCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = (readRegister(IP236x_REG_SYS_CTL8, errorCode) >> 4) & 0x0F;
    return value * 50;
}

void IP236x::setCellRechargeThreshold(uint16_t voltageDrop_mV, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t regValue = 0;

    if (voltageDrop_mV >= 400)
    {
        regValue = 3;
    }
    else
    {
        regValue = voltageDrop_mV / 50;
    }

    uint8_t currentValue = readRegister(IP236x_REG_SYS_CTL8, errorCode) & 0xF3;
    writeRegister(IP236x_REG_SYS_CTL8, currentValue | (regValue << 2), errorCode);
}

uint16_t IP236x::getCellRechargeThreshold(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = (readRegister(IP236x_REG_SYS_CTL8, errorCode) >> 2) & 0x03;
    return (value * 50);
}

// SYS_CTL9

void IP236x::enableStandbyMode(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL9, errorCode);
    value = bitWrite(value, 7, enable);
    writeRegister(IP236x_REG_SYS_CTL9, value, errorCode);
}

bool IP236x::isStandbyModeEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL9, errorCode) & (1 << 7);
}

void IP236x::enableBATLow(bool enable, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_SYS_CTL9, errorCode);
    value = bitWrite(value, 5, enable);
    writeRegister(IP236x_REG_SYS_CTL9, value, errorCode);
}

bool IP236x::isBATLowEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL9, errorCode) & (1 << 5);
}

// SYS_CTL10

void IP236x::setLowBatteryVoltage(uint16_t voltage_mV, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
}

uint16_t IP236x::getLowBatteryVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 4;
    return 0;
}

// SYS_CTL11

void IP236x::setOutputFeatures(bool enableDcDcOutput, bool enableVbusSrcDPdM, bool enableVbusSrcPd, bool enableVbusSrcSCP, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t currentValue = readRegister(IP236x_REG_SYS_CTL11, errorCode) & 0x0F;
    currentValue |= (enableDcDcOutput << 7) | (enableVbusSrcDPdM << 6) | (enableVbusSrcPd << 5) | (enableVbusSrcSCP << 4);
    writeRegister(IP236x_REG_SYS_CTL11, currentValue, errorCode);
}

bool IP236x::isDcDcOutputEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL11, errorCode) & 0x80;
}

bool IP236x::isVbusSrcDPdMEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL11, errorCode) & 0x40;
}

bool IP236x::isVbusSrcPdEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL11, errorCode) & 0x20;
}

bool IP236x::isVbusSrcSCPEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_SYS_CTL11, errorCode) & 0x10;
}

// TypeC_CTL8

void IP236x::setTypeCMode(TypeCMode mode, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_TypeC_CTL8, errorCode) & ~0xC0; 

    value |= (static_cast<uint8_t>(mode) << 6); 

    writeRegister(IP236x_REG_TypeC_CTL8, value, errorCode);
}

IP236x::TypeCMode IP236x::getTypeCMode(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return static_cast<TypeCMode>(readRegister(IP236x_REG_TypeC_CTL8, errorCode) >> 6); 
}

// TypeC_CTL9

void IP236x::enablePdoCurrentOutputSet(bool en5VPdoIset, bool en5VPdo3A, bool en9VPdoIset,
                                       bool en12VPdoIset, bool en15VPdoIset, bool en20VPdoIset,
                                       bool enPps1PdoIset, bool enPps2PdoIset, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = 0;
    value = bitWrite(value, 7, en5VPdo3A);
    value = bitWrite(value, 6, enPps2PdoIset);
    value = bitWrite(value, 5, enPps1PdoIset);
    value = bitWrite(value, 4, en20VPdoIset);
    value = bitWrite(value, 3, en15VPdoIset);
    value = bitWrite(value, 2, en12VPdoIset);
    value = bitWrite(value, 1, en9VPdoIset);
    value = bitWrite(value, 0, en5VPdoIset);
    writeRegister(IP236x_REG_TypeC_CTL9, value, errorCode);
}

bool IP236x::is5VPdo3AEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 7);
}

bool IP236x::isPps2PdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 6);
}

bool IP236x::isPps1PdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 5);
}

bool IP236x::is20VPdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 4);
}

bool IP236x::is15VPdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 3);
}

bool IP236x::is12VPdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 2);
}

bool IP236x::is9VPdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 1);
}

bool IP236x::is5VPdoIsetEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL9, errorCode) & (1 << 0);
}

void IP236x::writeTypeCCurrentSetting(uint8_t reg, uint16_t current_mA, uint16_t step, uint16_t maxCurrent, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    if (current_mA > maxCurrent)
        current_mA = maxCurrent;
    uint8_t value = current_mA / step;
    writeRegister(reg, value, errorCode);
}

// TypeC_CTL10 - TypeC_CTL14

void IP236x::setPDOCurrent5V(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL10, current_mA, 20, 3000, errorCode);
}

uint16_t IP236x::getPDOCurrent5V(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL10, errorCode) * 20;
}

void IP236x::setPDOCurrent9V(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL11, current_mA, 20, 3000, errorCode);
}

uint16_t IP236x::getPDOCurrent9V(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL11, errorCode) * 20;
}

void IP236x::setPDOCurrent12V(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL12, current_mA, 20, 3000, errorCode);
}

uint16_t IP236x::getPDOCurrent12V(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL12, errorCode) * 20;
}

void IP236x::setPDOCurrent15V(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL13, current_mA, 20, 3000, errorCode);
}

uint16_t IP236x::getPDOCurrent15V(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL13, errorCode) * 20;
}

void IP236x::setPDOCurrent20V(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL14, current_mA, 20, 5000, errorCode);
}

uint16_t IP236x::getPDOCurrent20V(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL14, errorCode) * 20;
}

// TypeC_CTL23 - TypeC_CTL24

void IP236x::setPDOCurrentPPS1(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL23, current_mA, 50, 5000, errorCode);
}

uint16_t IP236x::getPDOCurrentPPS1(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL23, errorCode) * 50;
}

void IP236x::setPDOCurrentPPS2(uint16_t current_mA, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    writeTypeCCurrentSetting(IP236x_REG_TypeC_CTL24, current_mA, 50, 5000, errorCode);
}

uint16_t IP236x::getPDOCurrentPPS2(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL24, errorCode) * 50;
}

// TypeC_CTL17

void IP236x::enableSrcPdo(bool en9VPdo, bool en12VPdo, bool en15VPdo, bool en20VPdo, bool enPps1Pdo, bool enPps2Pdo, uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_TypeC_CTL17, errorCode);
    value = bitWrite(value, 6, enPps2Pdo);
    value = bitWrite(value, 5, enPps1Pdo);
    value = bitWrite(value, 4, en20VPdo);
    value = bitWrite(value, 3, en15VPdo);
    value = bitWrite(value, 2, en12VPdo);
    value = bitWrite(value, 1, en9VPdo);
    writeRegister(IP236x_REG_TypeC_CTL17, value, errorCode);
}

bool IP236x::isSrcPdo9VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 1);
}

bool IP236x::isSrcPdo12VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 2);
}

bool IP236x::isSrcPdo15VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 3);
}

bool IP236x::isSrcPdo20VEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 4);
}

bool IP236x::isSrcPps1PdoEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 5);
}

bool IP236x::isSrcPps2PdoEnabled(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_CTL17, errorCode) & (1 << 6);
}

// STATE_CTL0

bool IP236x::isCharging(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL0, errorCode) & (1 << 5);
}

bool IP236x::isChargeFull(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL0, errorCode) & (1 << 4);
}

bool IP236x::isDischarging(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL0, errorCode) & (1 << 3);
}

IP236x::ChargeState IP236x::getChargeState(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t data = readRegister(IP236x_REG_STATE_CTL0, errorCode);
    uint8_t stateBits = data & 0x07;
    return static_cast<ChargeState>(stateBits);
}

// STATE_CTL1

bool IP236x::isFastCharge(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL1, errorCode) & (1 << 6);
}

// STATE_CTL2

bool IP236x::isVbusPresent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL2, errorCode) & (1 << 7);
}

bool IP236x::isVbusOvervoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL2, errorCode) & (1 << 6);
}

uint8_t IP236x::getChargeVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    uint8_t value = readRegister(IP236x_REG_STATE_CTL2, errorCode) & 0x07;
    switch (value)
    {
    case 0x07:
        value = 20;
        break;

    case 0x06:
        value = 15;
        break;

    case 0x05:
        value = 12;
        break;

    case 0x04:
        value = 9;
        break;

    case 0x03:
        value = 7;
        break;

    case 0x02:
        value = 5;
        break;
    default:
        value = 0;
        break;
    }
    return value;
}

// TypeC_STATE

bool IP236x::isTypeCSinkConnected(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 7);
}

bool IP236x::isTypeCSrcConnected(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 6);
}

bool IP236x::isTypeCSrcPdConnected(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 5);
}

bool IP236x::isTypeCSinkPdConnected(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 4);
}

bool IP236x::isVbusSinkQcActive(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 3);
}

bool IP236x::isVbusSrcQcActive(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_TypeC_STATE, errorCode) & (1 << 2);
}

// STATE_CTL3

bool IP236x::isVsysOverCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL3, errorCode) & (1 << 5);
}

bool IP236x::isVsysShortCircuitDt(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return readRegister(IP236x_REG_STATE_CTL3, errorCode) & (1 << 4);
}

// ADC

uint16_t IP236x::getVBATVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (((uint16_t)readRegister(IP236x_REG_BATVADC_DAT1, errorCode) << 8) | (uint16_t)readRegister(IP236x_REG_BATVADC_DAT0, errorCode));
}

uint16_t IP236x::getVsysVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (((uint16_t)readRegister(IP236x_REG_VsysVADC_DAT1, errorCode) << 8) | (uint16_t)readRegister(IP236x_REG_VsysVADC_DAT0, errorCode));
}

uint16_t IP236x::getBATCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (((uint16_t)readRegister(IP236x_REG_IBATIADC_DAT1, errorCode) << 8) | (uint16_t)readRegister(IP236x_REG_IBATIADC_DAT0, errorCode));
}

uint16_t IP236x::getVsysCurrent(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (((uint16_t)readRegister(IP236x_REG_IVsys_IADC_DAT1, errorCode) << 8) | (uint16_t)readRegister(IP236x_REG_ISYS_IADC_DAT0, errorCode));
}

uint32_t IP236x::getVsysPower(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (((uint32_t)readRegister(IP236x_REG_Vsys_POW_DAT1, errorCode) << 8) | (uint32_t)readRegister(IP236x_REG_Vsys_POW_DAT0, errorCode));
}

bool IP236x::isOverHeat(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return (readRegister(IP236x_REG_INTC_IADC_DAT0, errorCode) & (1 << 7));
}

// GPIO aka ADC raw readings

uint16_t IP236x::getNTCVoltage(uint8_t * errorCode)
{
    if (errorCode != nullptr) *errorCode = 0; // reset error code
    return ADC_TO_MV(((uint16_t)readRegister(IP236x_REG_VGPIO0_NTC_DAT1, errorCode) << 8) | (uint16_t)readRegister(IP236x_REG_VGPIO0_NTC_DAT0, errorCode));
}
