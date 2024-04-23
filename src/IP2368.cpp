#include "IP2368.h"

// Registers

// System Control Registers
#define IP2368_REG_SYS_CTL0 0x00  // Charge enable and other control settings
#define IP2368_REG_SYS_CTL1 0x01  // Battery series settings, battery type, current setting mode
#define IP2368_REG_SYS_CTL2 0x02  // Vset full-charge voltage setting
#define IP2368_REG_SYS_CTL3 0x03  // Iset charge power or current setting
#define IP2368_REG_SYS_CTL4 0x04  // Battery capacity setting
#define IP2368_REG_SYS_CTL6 0x06  // Current battery level
#define IP2368_REG_SYS_CTL7 0x07  // Trickle charge current, threshold and charge timeout setting
#define IP2368_REG_SYS_CTL8 0x08  // Stop charge current and recharge threshold setting
#define IP2368_REG_SYS_CTL9 0x09  // Standby enable and low battery voltage settings
#define IP2368_REG_SYS_CTL10 0x0A // Low battery voltage setting
#define IP2368_REG_SYS_CTL11 0x0B // Output enable register
#define IP2368_REG_SYS_CTL12 0x0C // Output maximum power selection register

// TYPE-C Control Registers
#define IP2368_REG_TypeC_CTL8 0x22  // TYPE-C mode control register
#define IP2368_REG_TypeC_CTL9 0x23  // Output Pdo current setting register
#define IP2368_REG_TypeC_CTL10 0x24 // 5VPdo current setting register
#define IP2368_REG_TypeC_CTL11 0x25 // 9VPdo current setting register
#define IP2368_REG_TypeC_CTL12 0x26 // 12VPdo current setting register
#define IP2368_REG_TypeC_CTL13 0x27 // 15VPdo current setting register
#define IP2368_REG_TypeC_CTL14 0x28 // 20VPdo current setting register
#define IP2368_REG_TypeC_CTL17 0x2B // Output Pdo setting register
#define IP2368_REG_TypeC_CTL23 0x29 // Pps1 Pdo current setting register
#define IP2368_REG_TypeC_CTL24 0x2A // Pps2 Pdo current setting register

// Read-only Status Indication Registers
#define IP2368_REG_SOC_CAP_DATA 0x30 // Battery percentage data register
#define IP2368_REG_STATE_CTL0 0x31   // Charge status control register
#define IP2368_REG_STATE_CTL1 0x32   // Charge status control register 2
#define IP2368_REG_STATE_CTL2 0x33   // Input Pd status control register
#define IP2368_REG_TypeC_STATE 0x34  // System status indication register
#define IP2368_REG_MOS_STATE 0x35    // Input MOS status indication register
#define IP2368_REG_STATE_CTL3 0x38   // System over-current indication register

// ADC Data Registers
#define IP2368_REG_BATVADC_DAT0 0x50         // VBAT voltage low 8 bits
#define IP2368_REG_BATVADC_DAT1 0x51         // VBAT voltage high 8 bits
#define IP2368_REG_VsysVADC_DAT0 0x52        // Vsys voltage low 8 bits
#define IP2368_REG_VsysVADC_DAT1 0x53        // Vsys voltage high 8 bits
#define IP2368_REG_IVbus_Sink_IADC_DAT0 0x54 // Input current low 8 bits
#define IP2368_REG_IVbus_Sink_IADC_DAT1 0x55 // Input current high 8 bits
#define IP2368_REG_IVbus_Src_IADC_DAT0 0x56  // Output current low 8 bits
#define IP2368_REG_IVbus_Src_IADC_DAT1 0x57  // Output current high 8 bits
#define IP2368_REG_IBATIADC_DAT0 0x6E        // BAT-end current low 8 bits
#define IP2368_REG_IBATIADC_DAT1 0x6F        // BAT-end current high 8 bits
#define IP2368_REG_ISYS_IADC_DAT0 0x70       // IVsys-end current low 8 bits
#define IP2368_REG_IVsys_IADC_DAT1 0x71      // IVsys-end current high 8 bits
#define IP2368_REG_Vsys_POW_DAT0 0x74        // Vsysterminal power low 8 bits
#define IP2368_REG_Vsys_POW_DAT1 0x75        // Vsysterminal power mid 8 bits
#define IP2368_REG_Vsys_POW_DAT2 0x76        // Vsysterminal power high 8 bits

// Additional ADC Data Registers for NTC, GPIOs
#define IP2368_REG_INTC_IADC_DAT0 0x77     // NTC output current setting
#define IP2368_REG_VGPIO0_NTC_DAT0 0x78    // VGPIO0_NTC ADC voltage low 8 bits
#define IP2368_REG_VGPIO0_NTC_DAT1 0x79    // VGPIO0_NTC ADC voltage high 8 bits
#define IP2368_REG_VGPIO1_Iset_DAT0 0x7A   // VGPIO1_Iset ADC voltage low 8 bits
#define IP2368_REG_VGPIO1_Iset_DAT1 0x7B   // VGPIO1_Iset ADC voltage high 8 bits
#define IP2368_REG_VGPIO2_Vset_DAT0 0x7C   // VGPIO2_Vset ADC voltage low 8 bits
#define IP2368_REG_VGPIO2_Vset_DAT1 0x7D   // VGPIO2_Vset ADC voltage high 8 bits
#define IP2368_REG_VGPIO3_FCAP_DAT0 0x7E   // VGPIO3_FCAP ADC voltage low 8 bits
#define IP2368_REG_VGPIO3_FCAP_DAT1 0x7F   // VGPIO3_FCAP ADC voltage high 8 bits
#define IP2368_REG_VGPIO4_BATNUM_DAT0 0x80 // VGPIO4_BATNUM ADC voltage low 8 bits
#define IP2368_REG_VGPIO4_BATNUM_DAT1 0x81 // VGPIO4_BATNUM ADC voltage high 8 bits

#define ADC_TO_MV(adc_val) ((uint16_t)((((uint32_t)(adc_val) * 3300) / 0xFFFF)))

void IP2368::begin()
{
#ifdef TwoWire_h
    Wire.begin();
#endif
}

uint8_t IP2368::writeRegister(uint8_t regAddress, uint8_t value)
{
#ifdef TwoWire_h
    Wire.beginTransmission(IP2368_address);
    delay (2);
    Wire.write(regAddress);
    delay (2);
    Wire.write(value);
    delay (2);
    uint8_t errorCode = Wire.endTransmission(); // 不發送停止信號
    delay (2);
    if (errorCode)
    {
        // Serial.print("Error during transmission: ");
        // Serial.println(errorCode);
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

uint8_t IP2368::readRegister(uint8_t regAddress)
{
#ifdef TwoWire_h
    Wire.beginTransmission(IP2368_address);
    delay (2);
    Wire.write(regAddress);
    delay (2);
    uint8_t errorCode = Wire.endTransmission(false); // 不發送停止信號
    delay (2);
    if (errorCode)
    {
        // Serial.print("Error during transmission: ");
        // Serial.println(errorCode);
        return -1;
    }

    uint8_t bytesRead = Wire.requestFrom(IP2368_address, (uint8_t)1, (bool)true); // 請求1個字節，並發送停止信號

    if (bytesRead != 1)
    {
        // Serial.println("Failed to read from device!");
        return -1;
    }
    return Wire.read();
#else
    return -1;
#endif
}

uint8_t IP2368::setBit(uint8_t value, uint8_t bit, bool enable)
{
    return (enable) ? value | (1 << bit) : value & ~(1 << bit);
}

// SYS_CTL0

void IP2368::enableCharger(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 0, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isChargerEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x01;
}

void IP2368::enableVbusSinkCtrl(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 1, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isVbusSinkCtrlEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x02;
}

void IP2368::enableVbusSinkSCP(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 2, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isVbusSinkSCPEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x04;
}

void IP2368::enableVbusSinkPD(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 3, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isVbusSinkPDEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x08;
}

void IP2368::enableVbusSinkDPdM(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 4, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isVbusSinkDPdMEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x10;
}

void IP2368::enableINTLow(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 5, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isINTLowEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x20;
}

void IP2368::ResetMCU(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 6, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

void IP2368::enableLoadOTP(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL0);
    value = setBit(value, 7, enable);
    writeRegister(IP2368_REG_SYS_CTL0, value);
}

bool IP2368::isLoadOTPEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL0) & 0x80;
}

// SYS_CTL1

void IP2368::enableBatteryTypeSetting(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    value = setBit(value, 3, enable);
    writeRegister(IP2368_REG_SYS_CTL1, value);
}

void IP2368::setBatteryType(BatteryType type)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1) & ~(1 << 2);

    value |= (type << 2);

    writeRegister(IP2368_REG_SYS_CTL1, value);
}

bool IP2368::isBatteryTypeSettingEnabled()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    return value & (1 << 3);
}

IP2368::BatteryType IP2368::getBatteryType()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    if (value & (1 << 2))
    {
        return BatteryType::LiIon;
    }
    else
    {
        return BatteryType::LiFePO4;
    }
}

void IP2368::enableCurrentOrPowerSettingMode(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    value = setBit(value, 1, enable);
    writeRegister(IP2368_REG_SYS_CTL1, value);
}

bool IP2368::isCurrentOrPowerSettingModeEnabled()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    return value & (1 << 1);
}

void IP2368::setCurrentOrPowerSettingMode(CurrentSettingMode mode)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    if (mode == CurrentSettingMode::BatteryCurrent)
    {
        value &= ~(1 << 0);
    }
    else
    {
        value |= (1 << 0);
    }
    writeRegister(IP2368_REG_SYS_CTL1, value);
}

IP2368::CurrentSettingMode IP2368::getPowerOrCurrentSettingMode()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL1);
    if (value & (1 << 0))
    {
        return CurrentSettingMode::InputPower;
    }
    else
    {
        return CurrentSettingMode::BatteryCurrent;
    }
}

// SYS_CTL2

void IP2368::enableFullChargeVoltageSetting(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL2);
    value = setBit(value, 7, enable);
    writeRegister(IP2368_REG_SYS_CTL2, value);
}

bool IP2368::isFullChargeVoltageSettingEnabled()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL2);
    return (value & (1 << 7)) != 0;
}

void IP2368::setFullChargeVoltage(uint16_t voltage)
{

    BatteryType batteryType = getBatteryType();

    uint16_t minVoltage, maxVoltage;
    if (batteryType == BatteryType::LiIon)
    {
        minVoltage = 4000;
        maxVoltage = 4400;
    }
    else
    {
        minVoltage = 3500;
        maxVoltage = 3700;
    }

    if (voltage < minVoltage)
        voltage = minVoltage;
    else if (voltage > maxVoltage)
        voltage = maxVoltage;

    uint8_t regValue = (voltage - minVoltage) / 10;
    writeRegister(IP2368_REG_SYS_CTL2, regValue);
}

uint16_t IP2368::getFullChargeVoltage()
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL2);
    BatteryType batteryType = getBatteryType();

    uint16_t baseVoltage = (batteryType == BatteryType::LiIon) ? 4000 : 3500;

    return baseVoltage + regValue * 10;
}

// SYS_CTL3

void IP2368::enablePowerOrCurrentSetting(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL3);
    value = setBit(value, 7, enable);
    writeRegister(IP2368_REG_SYS_CTL3, value);
}

bool IP2368::isPowerOrCurrentSettingEnabled()
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL3);
    return (value & (1 << 7)) != 0;
}

void IP2368::setMaxInputPowerOrBatteryCurrent(uint16_t value)
{
    bool isPowerMode = getPowerOrCurrentSettingMode() == CurrentSettingMode::InputPower;
    uint8_t regValue;

    if (isPowerMode)
    {
        uint16_t maxPower = 100;
        if (value > maxPower)
            value = maxPower;

        regValue = value;
    }
    else
    {
        uint16_t maxCurrent = 5000;
        if (value > maxCurrent)
            value = maxCurrent;

        regValue = value / 100;
    }

    writeRegister(IP2368_REG_SYS_CTL3, regValue & 0x7F);
}

uint16_t IP2368::getMaxInputPowerOrBatteryCurrent()
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL3) & 0x7F;
    bool isPowerMode = getPowerOrCurrentSettingMode() == CurrentSettingMode::InputPower;

    if (isPowerMode)
    {
        return regValue;
    }
    else
    {
        return regValue * 100;
    }
}

// SYS_CTL4

void IP2368::enableFullChargeCapacitySetting(bool enable)
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL4);
    regValue = setBit(regValue, 7, enable);
    writeRegister(IP2368_REG_SYS_CTL4, regValue);
}

bool IP2368::isFullChargeCapacitySettingEnabled()
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL4);
    return (regValue & (1 << 7)) != 0;
}

void IP2368::setFullChargeCapacity(uint16_t capacity)
{
    uint8_t maxCapacityRegisterValue = 0x7F;
    uint16_t maxCapacitymAh = maxCapacityRegisterValue * 200;

    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL4) & ~maxCapacityRegisterValue;

    if (capacity > maxCapacitymAh)
    {
        capacity = maxCapacitymAh;
    }

    regValue |= (capacity / 200) & maxCapacityRegisterValue;

    writeRegister(IP2368_REG_SYS_CTL4, regValue & 0x7F);
}

uint16_t IP2368::getFullChargeCapacity()
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL4) & 0x7F;
    return regValue * 200;
}

// SYS_CTL6

void IP2368::setCurrentBatteryCapacity(uint8_t capacity)
{
    writeRegister(IP2368_REG_SYS_CTL6, capacity);
}

uint8_t IP2368::getCurrentBatteryCapacity()
{
    return readRegister(IP2368_REG_SYS_CTL6);
}

// SYS_CTL7

void IP2368::setTrickleChargeCurrent(uint16_t current)
{
    uint8_t regValue = current / 50;
    if (regValue > 0xF)
    {
        regValue = 0xF;
    }

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL7) & 0x0F;
    writeRegister(IP2368_REG_SYS_CTL7, (regValue << 4) | currentValue);
}

uint16_t IP2368::getTrickleChargeCurrent()
{
    uint8_t regValue = (readRegister(IP2368_REG_SYS_CTL7) >> 4) & 0x0F;
    return static_cast<uint16_t>(regValue * 50);
}

void IP2368::setTrickleChargeVoltage(uint16_t voltage_mV)
{
    BatteryType batteryType = getBatteryType();
    uint8_t voltageSetting;

    uint16_t baseVoltage = (batteryType == BatteryType::LiFePO4) ? 2300 : 2800;
    uint16_t stepVoltage = 100;

    if (voltage_mV < baseVoltage)
    {
        voltage_mV = baseVoltage;
    }
    else if (voltage_mV > baseVoltage + stepVoltage * 3)
    {
        voltage_mV = baseVoltage + stepVoltage * 3;
    }

    voltageSetting = (voltage_mV - baseVoltage) / stepVoltage;

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL7) & 0xF3;
    writeRegister(IP2368_REG_SYS_CTL7, (voltageSetting << 2) | currentValue);
}

uint16_t IP2368::getTrickleChargeVoltage()
{
    BatteryType batteryType = getBatteryType();
    uint8_t voltageSetting = (readRegister(IP2368_REG_SYS_CTL7) >> 2) & 0x03;

    uint16_t baseVoltage = (batteryType == BatteryType::LiFePO4) ? 2300 : 2800;
    return baseVoltage + voltageSetting * 100;
}

void IP2368::setChargeTimeout(ChargeTimeout timeout)
{
    uint8_t timeoutSetting = static_cast<uint8_t>(timeout);

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL7) & 0xFC;
    writeRegister(IP2368_REG_SYS_CTL7, timeoutSetting | currentValue);
}

IP2368::ChargeTimeout IP2368::getChargeTimeout()
{
    uint8_t regValue = readRegister(IP2368_REG_SYS_CTL7) & 0x03;
    return static_cast<ChargeTimeout>(regValue);
}

// SYS_CTL8

void IP2368::setChargeStopCurrent(uint16_t current)
{
    if (current > 750)
        current = 750;
    uint8_t regValue = current / 50;

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL8) & 0x0F;
    writeRegister(IP2368_REG_SYS_CTL8, currentValue | (regValue << 4));
}

uint16_t IP2368::getChargeStopCurrent()
{
    uint8_t value = (readRegister(IP2368_REG_SYS_CTL8) >> 4) & 0x0F;
    return value * 50;
}

void IP2368::setCellRechargeThreshold(uint16_t voltageDrop_mV)
{
    uint8_t regValue = 0;

    if (voltageDrop_mV >= 400)
    {
        regValue = 3;
    }
    else
    {
        regValue = voltageDrop_mV / 100;
    }

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL8) & 0xF3;
    writeRegister(IP2368_REG_SYS_CTL8, currentValue | (regValue << 2));
}

uint16_t IP2368::getCellRechargeThreshold()
{
    uint8_t value = (readRegister(IP2368_REG_SYS_CTL8) >> 2) & 0x03;
    return value * 100 + 100;
}

// SYS_CTL9

void IP2368::enableStandbyMode(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL9);
    value = setBit(value, 7, enable);
    writeRegister(IP2368_REG_SYS_CTL9, value);
}

bool IP2368::isStandbyModeEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL9) & (1 << 7);
}

void IP2368::enableBATlowSet(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL9);
    value = setBit(value, 6, enable);
    writeRegister(IP2368_REG_SYS_CTL9, value);
}

bool IP2368::isBATlowSetEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL9) & (1 << 6);
}

void IP2368::enableBATLow(bool enable)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL9);
    value = setBit(value, 5, enable);
    writeRegister(IP2368_REG_SYS_CTL9, value);
}

bool IP2368::isBATLowEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL9) & (1 << 5);
}

// SYS_CTL10

void IP2368::setLowBatteryVoltage(uint16_t voltage_mV)
{
    BatteryType batteryType = getBatteryType();
    uint8_t voltageSetting;
    uint16_t stepVoltage = 100;

    uint16_t baseVoltage = (batteryType == BatteryType::LiFePO4) ? 2300 : 2800;

    if (voltage_mV < baseVoltage)
    {
        voltage_mV = baseVoltage;
    }
    else if (voltage_mV > baseVoltage + stepVoltage * 3)
    {
        voltage_mV = baseVoltage + stepVoltage * 3;
    }

    voltageSetting = (voltage_mV - baseVoltage) / stepVoltage;

    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL10) & 0x1F;
    writeRegister(IP2368_REG_SYS_CTL10, (voltageSetting << 5) | currentValue);
}

uint16_t IP2368::getLowBatteryVoltage()
{
    BatteryType batteryType = getBatteryType();
    uint8_t voltageSetting = (readRegister(IP2368_REG_SYS_CTL10) >> 5) & 0x03;

    uint16_t baseVoltage = (batteryType == BatteryType::LiFePO4) ? 2300 : 2800;
    return baseVoltage + voltageSetting * 100;
}

// SYS_CTL11

void IP2368::setOutputFeatures(bool enableDcDcOutput, bool enableVbusSrcDPdM, bool enableVbusSrcPd, bool enableVbusSrcSCP)
{
    uint8_t currentValue = readRegister(IP2368_REG_SYS_CTL11) & 0x0F;
    currentValue |= (enableDcDcOutput << 7) | (enableVbusSrcDPdM << 6) | (enableVbusSrcPd << 5) | (enableVbusSrcSCP << 4);
    writeRegister(IP2368_REG_SYS_CTL11, currentValue);
}

bool IP2368::isDcDcOutputEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL11) & 0x80;
}

bool IP2368::isVbusSrcDPdMEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL11) & 0x40;
}

bool IP2368::isVbusSrcPdEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL11) & 0x20;
}

bool IP2368::isVbusSrcSCPEnabled()
{
    return readRegister(IP2368_REG_SYS_CTL11) & 0x10;
}

// SYS_CTL12

void IP2368::setMaxOutputPower(Vbus1OutputPower power)
{
    uint8_t value = readRegister(IP2368_REG_SYS_CTL12) & 0x1F; 
    value |= static_cast<uint8_t>(power) << 5;                 
    writeRegister(IP2368_REG_SYS_CTL12, value);
}

IP2368::Vbus1OutputPower IP2368::getMaxOutputPower()
{
    return static_cast<Vbus1OutputPower>((readRegister(IP2368_REG_SYS_CTL12) >> 5) & 0x03); 
}

// TypeC_CTL8

void IP2368::setTypeCMode(TypeCMode mode)
{
    uint8_t value = readRegister(IP2368_REG_TypeC_CTL8) & ~0xC0; 

    value |= (mode << 6); 

    writeRegister(IP2368_REG_TypeC_CTL8, value);
}

IP2368::TypeCMode IP2368::getTypeCMode()
{
    return static_cast<TypeCMode>(readRegister(IP2368_REG_TypeC_CTL8) >> 6); 
}

// TypeC_CTL9

void IP2368::enablePdoCurrentOutputSet(bool en5VPdoIset, bool en5VPdo3A, bool en9VPdoIset,
                                       bool en12VPdoIset, bool en15VPdoIset, bool en20VPdoIset,
                                       bool enPps1PdoIset, bool enPps2PdoIset)
{
    uint8_t value = 0;
    value = setBit(value, 7, en5VPdo3A);
    value = setBit(value, 6, enPps2PdoIset);
    value = setBit(value, 5, enPps1PdoIset);
    value = setBit(value, 4, en20VPdoIset);
    value = setBit(value, 3, en15VPdoIset);
    value = setBit(value, 2, en12VPdoIset);
    value = setBit(value, 1, en9VPdoIset);
    value = setBit(value, 0, en5VPdoIset);
    writeRegister(IP2368_REG_TypeC_CTL9, value);
}

bool IP2368::is5VPdo3AEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 7);
}

bool IP2368::isPps2PdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 6);
}

bool IP2368::isPps1PdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 5);
}

bool IP2368::is20VPdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 4);
}

bool IP2368::is15VPdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 3);
}

bool IP2368::is12VPdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 2);
}

bool IP2368::is9VPdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 1);
}

bool IP2368::is5VPdoIsetEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL9) & (1 << 0);
}

void IP2368::writeTypeCCurrentSetting(uint8_t reg, uint16_t current_mA, uint16_t step, uint16_t maxCurrent)
{
    if (current_mA > maxCurrent)
        current_mA = maxCurrent;
    uint8_t value = current_mA / step;
    writeRegister(reg, value);
}

// TypeC_CTL10 - TypeC_CTL14

void IP2368::setPDOCurrent5V(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL10, current_mA, 20, 3000);
}

uint16_t IP2368::getPDOCurrent5V()
{
    return readRegister(IP2368_REG_TypeC_CTL10) * 20;
}

void IP2368::setPDOCurrent9V(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL11, current_mA, 20, 3000);
}

uint16_t IP2368::getPDOCurrent9V()
{
    return readRegister(IP2368_REG_TypeC_CTL11) * 20;
}

void IP2368::setPDOCurrent12V(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL12, current_mA, 20, 3000);
}

uint16_t IP2368::getPDOCurrent12V()
{
    return readRegister(IP2368_REG_TypeC_CTL12) * 20;
}

void IP2368::setPDOCurrent15V(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL13, current_mA, 20, 3000);
}

uint16_t IP2368::getPDOCurrent15V()
{
    return readRegister(IP2368_REG_TypeC_CTL13) * 20;
}

void IP2368::setPDOCurrent20V(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL14, current_mA, 20, 5000);
}

uint16_t IP2368::getPDOCurrent20V()
{
    return readRegister(IP2368_REG_TypeC_CTL14) * 20;
}

// TypeC_CTL23 - TypeC_CTL24

void IP2368::setPDOCurrentPPS1(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL23, current_mA, 50, 5000);
}

uint16_t IP2368::getPDOCurrentPPS1()
{
    return readRegister(IP2368_REG_TypeC_CTL23) * 50;
}

void IP2368::setPDOCurrentPPS2(uint16_t current_mA)
{
    writeTypeCCurrentSetting(IP2368_REG_TypeC_CTL24, current_mA, 50, 5000);
}

uint16_t IP2368::getPDOCurrentPPS2()
{
    return readRegister(IP2368_REG_TypeC_CTL24) * 50;
}

// TypeC_CTL17

void IP2368::enableSrcPdo(bool en9VPdo, bool en12VPdo, bool en15VPdo, bool en20VPdo, bool enPps1Pdo, bool enPps2Pdo)
{
    uint8_t value = readRegister(IP2368_REG_TypeC_CTL17);
    value = setBit(value, 6, enPps2Pdo);
    value = setBit(value, 5, enPps1Pdo);
    value = setBit(value, 4, en20VPdo);
    value = setBit(value, 3, en15VPdo);
    value = setBit(value, 2, en12VPdo);
    value = setBit(value, 1, en9VPdo);
    writeRegister(IP2368_REG_TypeC_CTL17, value);
}

bool IP2368::isSrcPdo9VEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 1);
}

bool IP2368::isSrcPdo12VEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 2);
}

bool IP2368::isSrcPdo15VEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 3);
}

bool IP2368::isSrcPdo20VEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 4);
}

bool IP2368::isSrcPps1PdoEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 5);
}

bool IP2368::isSrcPps2PdoEnabled()
{
    return readRegister(IP2368_REG_TypeC_CTL17) & (1 << 6);
}

// SOC_CAP_DATA

uint8_t IP2368::getBatteryPercentage()
{
    return readRegister(IP2368_REG_SOC_CAP_DATA);
}

// STATE_CTL0

bool IP2368::isCharging()
{
    return readRegister(IP2368_REG_STATE_CTL0) & (1 << 5);
}

bool IP2368::isChargeFull()
{
    return readRegister(IP2368_REG_STATE_CTL0) & (1 << 4);
}

bool IP2368::isDischarging()
{
    return readRegister(IP2368_REG_STATE_CTL0) & (1 << 3);
}

IP2368::ChargeState IP2368::getChargeState()
{
    uint8_t data = readRegister(IP2368_REG_STATE_CTL0);
    uint8_t stateBits = data & 0x07;
    return static_cast<ChargeState>(stateBits);
}

// STATE_CTL1

bool IP2368::isFastCharge()
{
    return readRegister(IP2368_REG_STATE_CTL1) & (1 << 6);
}

// STATE_CTL2

bool IP2368::isVbusPresent()
{
    return readRegister(IP2368_REG_STATE_CTL2) & (1 << 7);
}

bool IP2368::isVbusOvervoltage()
{
    return readRegister(IP2368_REG_STATE_CTL2) & (1 << 6);
}

uint8_t IP2368::getChargeVoltage()
{
    uint8_t value = readRegister(IP2368_REG_STATE_CTL2) & 0x07;
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

bool IP2368::isTypeCSinkConnected()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 7);
}

bool IP2368::isTypeCSrcConnected()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 6);
}

bool IP2368::isTypeCSrcPdConnected()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 5);
}

bool IP2368::isTypeCSinkPdConnected()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 4);
}

bool IP2368::isVbusSinkQcActive()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 3);
}

bool IP2368::isVbusSrcQcActive()
{
    return readRegister(IP2368_REG_TypeC_STATE) & (1 << 2);
}

// MOS_STATE

bool IP2368::isVbusMosStateOpen()
{
    return readRegister(IP2368_REG_MOS_STATE) & (1 << 6);
}

// STATE_CTL3

bool IP2368::isVsysOverCurrent()
{
    return readRegister(IP2368_REG_STATE_CTL3) & (1 << 5);
}

bool IP2368::isVsysSdortCircuitDt()
{
    return readRegister(IP2368_REG_STATE_CTL3) & (1 << 4);
}

// ADC

uint16_t IP2368::getVBATVoltage()
{
    return (((uint16_t)readRegister(IP2368_REG_BATVADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_BATVADC_DAT0));
}

uint16_t IP2368::getVsysVoltage()
{
    return (((uint16_t)readRegister(IP2368_REG_VsysVADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VsysVADC_DAT0));
}

uint16_t IP2368::getChargeInputCurrent()
{
    return (((uint16_t)readRegister(IP2368_REG_IVbus_Sink_IADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_IVbus_Sink_IADC_DAT0));
}

uint16_t IP2368::getDischargeOutputCurrent()
{
    return (((uint16_t)readRegister(IP2368_REG_IVbus_Src_IADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_IVbus_Src_IADC_DAT0));
}

uint16_t IP2368::getBATCurrent()
{
    return (((uint16_t)readRegister(IP2368_REG_IBATIADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_IBATIADC_DAT0));
}

uint16_t IP2368::getVsysCurrent()
{
    return (((uint16_t)readRegister(IP2368_REG_IVsys_IADC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_ISYS_IADC_DAT0));
}

uint32_t IP2368::getVsysPower()
{
    return (((uint32_t)readRegister(IP2368_REG_Vsys_POW_DAT2) << 16) | ((uint32_t)readRegister(IP2368_REG_Vsys_POW_DAT1) << 8) | (uint32_t)readRegister(IP2368_REG_Vsys_POW_DAT0));
}

bool IP2368::isOverHeat()
{
    return (readRegister(IP2368_REG_INTC_IADC_DAT0) & (1 << 7));
}

// GPIO aka ADC raw readings

uint16_t IP2368::getNTCVoltage()
{
    return ADC_TO_MV(((uint16_t)readRegister(IP2368_REG_VGPIO0_NTC_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VGPIO0_NTC_DAT0));
}

uint16_t IP2368::getCurrentSettingVoltage()
{
    return ADC_TO_MV(((uint16_t)readRegister(IP2368_REG_VGPIO1_Iset_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VGPIO1_Iset_DAT0));
}

uint16_t IP2368::getVoltageSettingVoltage()
{
    return ADC_TO_MV(((uint16_t)readRegister(IP2368_REG_VGPIO2_Vset_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VGPIO2_Vset_DAT0));
}

uint16_t IP2368::getFCAPVoltage()
{
    return ADC_TO_MV(((uint16_t)readRegister(IP2368_REG_VGPIO3_FCAP_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VGPIO3_FCAP_DAT0));
}

uint16_t IP2368::getBatteryCountVoltage()
{
    return ADC_TO_MV(((uint16_t)readRegister(IP2368_REG_VGPIO4_BATNUM_DAT1) << 8) | (uint16_t)readRegister(IP2368_REG_VGPIO4_BATNUM_DAT0));
}
