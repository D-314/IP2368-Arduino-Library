#ifndef IP2368_H
#define IP2368_H

#include "IP236x.h"

class IP2368 : public IP236x {
public:
    using IP236x::IP236x;

    // Enumeration for defining the type of battery
    enum class BatteryType
    {
        LiFePO4 = 0, // Lithium iron phosphate batteries
        LiIon = 1    // Lithium ion batteries
    };

    // Enumeration for defining the current setting mode
    enum class CurrentSettingMode
    {
        BatteryCurrent = 0, // Setting the battery current
        InputPower = 1      // Setting the input power
    };

    // Enumeration for selecting Vbus1 output power
    enum class Vbus1OutputPower
    {
        W20 = 0,
        W25 = 1,
        W30 = 2,
        W45 = 3,
        W60 = 4,
        W100 = 5
    };

    // Enumeration for setting the charge timeout
    enum class ChargeTimeout
    {
        Disabled = 0,
        H24 = 1,
        H36 = 2,
        H48 = 3
    };


    ///////// IS? ////////

    // SYS_CTL0

    bool isVbusSinkCtrlEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL1

    bool isBatteryTypeSettingEnabled(uint8_t * errorCode = nullptr);
    bool isCurrentOrPowerSettingModeEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL2

    bool isFullChargeVoltageSettingEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL3

    bool isPowerOrCurrentSettingEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL4

    bool isFullChargeCapacitySettingEnabled(uint8_t * errorCode = nullptr);

    // SYS_CTL9
    bool isBATlowSetEnabled(uint8_t * errorCode = nullptr);

    // TypeC_STATE

    // MOS_STATE
    bool isVbusMosStateOpen(uint8_t * errorCode = nullptr);

    ///////// SET ////////

    // SYS_CTL0

    void enableVbusSinkCtrl(bool enable = true, uint8_t * errorCode = nullptr);

    // SYS_CTL1

    void enableBatteryTypeSetting(bool enable, uint8_t * errorCode = nullptr);
    void setBatteryType(BatteryType type, uint8_t * errorCode = nullptr);
    void enableCurrentOrPowerSettingMode(bool enable, uint8_t * errorCode = nullptr);
    void setCurrentOrPowerSettingMode(CurrentSettingMode mode, uint8_t * errorCode = nullptr);

    // SYS_CTL2

    void setFullChargeVoltage(uint16_t voltage, uint8_t * errorCode = nullptr) override;
    void enableFullChargeVoltageSetting(bool enable, uint8_t * errorCode = nullptr);

    // SYS_CTL3

    void enablePowerOrCurrentSetting(bool enable, uint8_t * errorCode = nullptr);
    void setMaxInputPowerOrBatteryCurrent(uint16_t value, uint8_t * errorCode = nullptr) override;

    // SYS_CTL4

    void enableFullChargeCapacitySetting(bool enable, uint8_t * errorCode = nullptr);
    void setFullChargeCapacity(uint16_t capacity, uint8_t * errorCode = nullptr);

    // SYS_CTL6

    void setCurrentBatteryCapacity(uint8_t capacity, uint8_t * errorCode = nullptr);

    // SYS_CTL7

    void setTrickleChargeCurrent(uint16_t current = 200, uint8_t * errorCode = nullptr) override;
    void setTrickleChargeVoltage(uint16_t voltage_mv, uint8_t * errorCode = nullptr);
    void setChargeTimeout(ChargeTimeout timeout = ChargeTimeout::H36, uint8_t * errorCode = nullptr);

    // SYS_CTL9

    void enableBATlowSet(bool enable, uint8_t * errorCode = nullptr);

    // SYS_CTL10

    void setLowBatteryVoltage(uint16_t voltage_mV, uint8_t * errorCode = nullptr) override;

    // SYS_CTL12

    void setMaxOutputPower(Vbus1OutputPower power = Vbus1OutputPower::W100, uint8_t * errorCode = nullptr);

    // SOC_CAP_DATA

    void setBatteryPercentage(uint8_t battery_level, uint8_t * errorCode = nullptr);

    ///////// GET ////////

    // SYS_CTL1

    BatteryType getBatteryType(uint8_t * errorCode = nullptr);
    CurrentSettingMode getPowerOrCurrentSettingMode(uint8_t * errorCode = nullptr);

    // SYS_CTL2

    uint16_t getFullChargeVoltage(uint8_t * errorCode = nullptr) override;

    // SYS_CTL3

    uint16_t getMaxInputPowerOrBatteryCurrent(uint8_t * errorCode = nullptr) override;

    // SYS_CTL4

    uint16_t getFullChargeCapacity(uint8_t * errorCode = nullptr);

    // SYS_CTL6

    uint8_t getCurrentBatteryCapacity(uint8_t * errorCode = nullptr);

    // SYS_CTL7

    uint16_t getTrickleChargeCurrent(uint8_t * errorCode = nullptr) override;
    uint16_t getTrickleChargeVoltage(uint8_t * errorCode = nullptr);
    ChargeTimeout getChargeTimeout(uint8_t * errorCode = nullptr);

    // SYS_CTL10

    uint16_t getLowBatteryVoltage(uint8_t * errorCode = nullptr) override;

    // SYS_CTL12

    Vbus1OutputPower getMaxOutputPower(uint8_t * errorCode = nullptr);

    // SOC_CAP_DATA

    uint8_t getBatteryPercentage(uint8_t * errorCode = nullptr);

    // ADC
    uint16_t getChargeInputCurrent(uint8_t * errorCode = nullptr);
    uint16_t getDischargeOutputCurrent(uint8_t * errorCode = nullptr);

    // GPIO

    uint16_t getCurrentSettingVoltage(uint8_t * errorCode = nullptr);
    uint16_t getVoltageSettingVoltage(uint8_t * errorCode = nullptr);
    uint16_t getFCAPVoltage(uint8_t * errorCode = nullptr);
    uint16_t getBatteryCountVoltage(uint8_t * errorCode = nullptr);

};

#endif
